/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George Todoran <todorangrg@gmail.com>   *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#ifndef TUW_NOISE_SIM_MODEL_H
#define TUW_NOISE_SIM_MODEL_H

#include <cmath>
#include <math.h>
#include <random>


#include <sdf/sdf.hh>

namespace tuw {
    
class NoiseSimModel {

/// \brief Constructor    
public: NoiseSimModel();
/// \brief Destructor
public: ~NoiseSimModel();

/// \brief loads noise model parameters from SDF element; @return if noise block is found in sdf
public: bool loadParam ( sdf::ElementPtr _sdf );

/**
 * advances the measurement noise model and applies noise on the input
 * @see http://www.mdpi.com/1424-8220/15/3/5293
 * @param _xI ground-truth measurement
 * @param _dt time since last measurement
 * @return new noised measurement
 **/
public: double sim ( const double _xI, const double _dt );
/// \brief returns final measurement state variable
public: const double x () const;
/// \brief returns standard deviation of white noise term
public: const double sigmaWhiteNoise () const;
/// \brief resets all state variables
public: void reset ();
    
/// \brief Random generator device
private: std::random_device randGen_;
/// \brief Unary white noise
private: std::normal_distribution<double> randGaussUnit_;
/// \brief Sensor white noise
private: std::normal_distribution<double> randGaussWn_;

//internal state variables
/// \brief final measurement state variable
private: double xSim_;
/// \brief non-noised measurement state variable (through bandwidth 1st order filter)
private: double xBw_;
/// \brief Gauss-Markov bias term
private: double bGm_;
/// \brief random walk bias term
private: double bRw_;

//constant parameters
/// \brief sensor white noise mean
private: double meanWn_;
/// \brief sensor white noise standard deviation
private: double sigWn_;
/// \brief sensor bandwidth time constant
private: double tauBw_;
/// \brief Gauss-Markov process frequency ( = 1 / time constant )
private: double fGm_;
/// \brief Gauss-Markov process standard deviation
private: double sigGm_;
/// \brief random walk process standard deviation
private: double sigRw_;

/// \brief flag for quantisation noise
private: bool quantized_;
/// \brief quantization precision
private: double precision_;
};
    
};

#endif //TUW_NOISE_SIM_MODEL_H
