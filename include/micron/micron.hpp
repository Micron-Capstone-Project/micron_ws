#pragma once

#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <jacl/jacl.hpp>
#include <serial_wrapper/serial_wrapper.hpp>
#include <mqtt_handler/mqtt_handler.hpp>

#define MICRON_DEBUG
#define MICRON_ENABLE_MQTT

namespace micron{

//-- add enable_if
namespace detail{
    template <typename T>
    auto serialize(T t) -> std::string{
        return std::to_string(t);
    }

    template <>
    auto serialize(const char* t) -> std::string{
        return std::string(t);
    }

    template <typename T, typename ...Ts>
    auto serialize(T t, Ts... ts) -> std::string{
        return serialize(t) + serialize(ts...);
    }
    constexpr static auto RPS2RPM(60);
}

class Micron{
public:
    Micron()
        : username_("C_Project")
        , key_("aio_iBqz42cYNefQOmuypkcvzv7ya0mw")
        , topic_("sensor-1")
        , mqtt_(username_, key_)
        , sw_("/dev/ttyACM0", micron::baud_rate_t(1000000))
        , plant_(&plant_ss_, SAMPLING_RATE)
        , sifd_(&plant_, {10.,10.,10.})
        , kpos_(&kpos_ss_, SAMPLING_RATE)
        , kspd_(&kspd_ss_, SAMPLING_RATE){

        transmit_thread_ = boost::thread{boost::bind(&Micron::transmitThread, this)};

        jacl::parser::readStateSpace(&kpos_ss_, "../config/position_controller.jacl");
        jacl::parser::readStateSpace(&kspd_ss_, "../config/speed_controller.jacl");
        jacl::parser::readStateSpace(&plant_ss_, "../config/motor_dc_dsimo.jacl");
        sifd_.init({{-.11,-.15}});
    }
    ~Micron(){
        transmit_thread_.join();
    }

    auto go();
    auto stop(){
        is_running_ = false;
        mqtt_.disconnect();
        std::ofstream pos_file("../sample/position_sample.txt");
        std::ofstream spd_file("../sample/speed_sample.txt");
        std::ofstream cur_file("../sample/current_sample.txt");
        std::ofstream err_pos_file("../sample/error_position_sample.txt");
        std::ofstream err_spd_file("../sample/error_speed_sample.txt");
        std::ofstream err_cur_file("../sample/error_current_sample.txt");
        for(const auto& p:response_sample_){
            pos_file << getPosition(p) << "\n";
            spd_file << getSpeed(p) << "\n";
            cur_file << getCurrent(p) << "\n";
        }
        for(const auto& p:error_sample_){
            err_pos_file << getPosition(p) << "\n";
            err_spd_file << getSpeed(p) << "\n";
            err_cur_file << getCurrent(p) << "\n";
        }
        pos_file.close();
        spd_file.close();
        cur_file.close();
    }

private:    
    auto genFault(double _val, double _bias, double _dead_zone, double _scale){
        auto res(_val); 
        res += _bias;
        if(std::fabs(res) < _dead_zone)
            res = .0;
        res *= _scale;
        return res;
    }
    void transmitThread();

private:
    //-- MQTT sutff
    std::string username_;
    std::string key_;
    std::string topic_;
    const unsigned int MQTT_DELAY{6000}; // in milliseconds
    std::string msg_;
    MQTTHandler mqtt_;
    boost::thread transmit_thread_;
    std::atomic<bool> is_running_; 

    //-- Serial comm stuff
    SerialWrapper<> sw_;   
    //-- Sampling stuff
    static constexpr std::size_t MAX_SAMPLE_DATA{100};
    using sensor_pack_t = std::tuple<double,double,double>;
    std::vector<sensor_pack_t> response_sample_;
    std::vector<sensor_pack_t> error_sample_;
    inline double getPosition(const sensor_pack_t& _pack){return std::get<0>(_pack);}
    inline double getSpeed(const sensor_pack_t& _pack){return std::get<1>(_pack);}
    inline double getCurrent(const sensor_pack_t& _pack){return std::get<2>(_pack);}

    //-- JACL or controller stuff0
    const unsigned int SAMPLING_RATE{100}; // in microseconds, not in Hz
    //-- DC motor open-loop
    using PlantSS = jacl::state_space::Linear<double,3, 1, 3>;
    PlantSS plant_ss_;
    using PlantSys = jacl::system::Discrete<PlantSS>;
    PlantSys plant_;
    //-- SIFD
    static constexpr std::size_t DEDICATED_STATE{0};
    jacl::diagnosis::SIFD<PlantSys, DEDICATED_STATE> sifd_;
    //-- Position controller
    using KPosSS = jacl::state_space::Linear<double,3,1,1>;
    KPosSS kpos_ss_;
    using KPos = jacl::system::Discrete<KPosSS>;
    KPos kpos_;
    //-- Speed controller
    using KSpdSS = jacl::state_space::Linear<double,3,1,1>;
    KSpdSS kspd_ss_;
    using KSpd = jacl::system::Discrete<KSpdSS>;
    KSpd kspd_;
      
};

auto Micron::go(){
    is_running_ = true;
    arma::mat ref(1,1,arma::fill::zeros);
    arma::mat err(ref);
    arma::mat in(ref);
    arma::mat out(3,1,arma::fill::zeros);
    ref(0) = 90.0; // degrees
    std::tuple<bool,bool,bool> sensor_fault(std::make_tuple(false,false,false));
    bool gen_fault(false);
    while(is_running_){
//        err = ref - arma::vec({pres_data_[0]});
//        in = k_pos_.propagate(err);
//        obsv_.setInput(in);
                                          
        std::string out_serialized( sw_.getData() );
        // std::cout << "Raw : " << out_serialized << std::endl;
        // out_serialized = "{\"position\":\"12.00\",\"speed\":\"0.00\",\"current\":\"4.40\",\"voltage\":\"0.00\"}";
        if(out_serialized !=  ""){
            std::istringstream out_json( out_serialized );
            boost::property_tree::ptree out_ptree;
            try{
                boost::property_tree::read_json(out_json, out_ptree);
                out(0) = out_ptree.get<double>("position");
                out(1) = out_ptree.get<double>("speed") * detail::RPS2RPM;
                out(2) = out_ptree.get<double>("current");
                in(0) = out_ptree.get<double>("voltage");
                       
                #ifdef MICRON_DEBUG
                std::cout << "[MICRON] Received : " << out_serialized << std::endl;            
                #endif        
                if(gen_fault){
                    out(0) = genFault(out(0), 0., 0., 1.);
                    out(1) = genFault(out(1), 0., 0., 1.);
                    out(2) = genFault(out(2), 0., 0., 1.);
                }
                jacl::diagnosis::SIFD<PlantSys, DEDICATED_STATE>::diag_pack_t diag_pack = sifd_.detect(in, out);
                std::get<0>(sensor_fault) |= std::get<2>(diag_pack[0]);
                std::get<1>(sensor_fault) |= std::get<2>(diag_pack[1]);
                std::get<2>(sensor_fault) |= std::get<2>(diag_pack[2]);

                if(response_sample_.size() < MAX_SAMPLE_DATA){
                    response_sample_.emplace_back(std::make_tuple(out(0),out(1),out(2)));
                    error_sample_.emplace_back(std::make_tuple(std::get<1>(diag_pack[0])(0),std::get<1>(diag_pack[1])(0),std::get<1>(diag_pack[2])(0)));

                    // if(response_sample_.size() > MAX_SAMPLE_DATA/4)
                    //     gen_fault = true;

                    // if(response_sample_.size() > 0)
                    //     gen_fault = true;

                    if(response_sample_.size() == MAX_SAMPLE_DATA)
                        std::cout << "[MICRON] Sampling finished." << std::endl;
                }   

                //-- Send input to arduino
                boost::property_tree::ptree in_ptree;
                in_ptree.put("voltage", std::round(in(0)*1000.)*0.001);
                std::ostringstream in_json;
                boost::property_tree::write_json(in_json, in_ptree, false);        
                // sw_.sendData(in_json.str());
                #ifdef MICRON_DEBUG
                std::cout << "[MICRON] Transmitted Arduino msg : " << in_json.str();
                #endif
                //-- prepare to publish to MQTT
                #ifdef MICRON_ENABLE_MQTT
                msg_ = detail::serialize("value1: ",out(0),
                        ";status1: ",std::get<0>(sensor_fault),
                        ";value2: ",out(1),
                        ";status2: ",std::get<1>(sensor_fault),
                        ";value3: ",out(2),
                        ";status3: ",std::get<2>(sensor_fault));
                #ifdef MICRON_DEBUG
                std::cout << "[MICRON] MQTT msg : " << msg_ << std::endl;
                #endif
                #endif
            }catch(const boost::property_tree::ptree_error& e){
                std::cerr << "[MICRON] Failed to parse." << std::endl;
            }
        }
        boost::this_thread::sleep_for(boost::chrono::microseconds{SAMPLING_RATE});
    }
}

void Micron::transmitThread(){
    while(mqtt_.isConnected()){
        mqtt_.send(username_+"/feeds/"+topic_, msg_);
        boost::this_thread::sleep_for(boost::chrono::milliseconds{MQTT_DELAY});
    }
}

}
