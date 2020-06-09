/**
*    @author : koseng (Lintang)
*    @brief : discrete system
*/
#pragma once

#include <BasicLinearAlgebra.h>

namespace micron{
template<size_t n_states,
         size_t n_inputs,
         size_t n_outputs>
class DiscreteSystem{
public:
  typedef BLA::Matrix<n_states,n_states> state_matrix_t;
  typedef BLA::Matrix<n_states,n_inputs> input_matrix_t;
  typedef BLA::Matrix<n_outputs,n_states> output_matrix_t;
  typedef BLA::Matrix<n_outputs,n_inputs> feedforward_matrix_t;
  typedef BLA::Matrix<n_states> state_t;
  typedef BLA::Matrix<n_inputs> input_t;
  typedef BLA::Matrix<n_outputs> output_t;
  
public:
  DiscreteSystem(const state_matrix_t& _A,
                 const input_matrix_t& _B,
                 const output_matrix_t& _C,
                 const feedforward_matrix_t& _D)
      : A_(_A)
      , B_(_B)
      , C_(_C)
      , D_(_D){
    state_.Fill(0);
    prev_state_.Fill(0);
    input_.Fill(0);
    output_.Fill(0);
  }
  ~DiscreteSystem(){
    
  }
  inline output_t convolve(const input_t& _input){
    setInput(_input);
    state_ = dstate();
    output_ = output();
    prev_state_ = state_;
    return output_;
  }
  
private:
  inline void setInput(const input_t& _input){
    input_ = _input;
  }
  inline state_t dstate(){
    return A_*prev_state_ + B_*input_;
  }
  inline output_t output(){
    return C_*prev_state_ + D_*input_;
  }
  
private:
  state_matrix_t A_;
  input_matrix_t B_;
  output_matrix_t C_;
  feedforward_matrix_t D_;
  state_t state_;
  state_t prev_state_;
  input_t input_;
  output_t output_;
  
};
}
