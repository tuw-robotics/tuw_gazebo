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

#include <float.h>
#include <string.h>
#include <math.h>

#include <tuw_gazebo_plugins/noise_sim_model.h>

using namespace std;
using namespace tuw;

NoiseSimModel::NoiseSimModel () :
    xSim_  ( 0 ),
    xBw_   ( 0 ),
    bGm_   ( 0 ),
    bRw_   ( 0 ),
    meanWn_( 0 ),
    sigWn_ ( 0 ),
    tauBw_ ( 0 ),
    fGm_   ( 0 ),
    sigGm_ ( 0 ),
    sigRw_ ( 0 ),
    quantized_(false) {
	randGaussWn_ = normal_distribution<double>( meanWn_, sigWn_ );
}
NoiseSimModel::~NoiseSimModel () {

}
void NoiseSimModel::reset () {
    xSim_ = 0;
    xBw_  = 0;
    bGm_  = 0;
    bRw_  = 0;
}
const double NoiseSimModel::x () const {
    return xSim_;
}
const double NoiseSimModel::sigmaWhiteNoise() const {
    return sigWn_;
}

double NoiseSimModel::sim ( const double _xI, const double _dt ) {
    if( fabs(_dt) > FLT_MIN ) {
	double alpha = _dt / (tauBw_ + _dt);
	double beta  = _dt *  fGm_;
	
	xBw_ = (1. - alpha) * xBw_ + alpha  * _xI;
	bGm_ = (1. - beta ) * bGm_ + sigGm_ * sqrt( 1. - exp( - 2 * beta) ) * randGaussUnit_(randGen_);
	bRw_ =                bRw_ + sigRw_ * sqrt( _dt )                   * randGaussUnit_(randGen_);
	
	xSim_ = xBw_ + bGm_ + bRw_ + randGaussWn_(randGen_);
	
	if (quantized_) { xSim_ = round(xSim_ / precision_) * precision_; } 
    }
    return xSim_;
}

bool NoiseSimModel::loadParam ( sdf::ElementPtr _sdf ) {
    bool foundSimDescription = false;
    if ( _sdf->HasElement("noise") ) {
	if( _sdf->GetElement("noise")->HasAttribute("type") ) {
	    if( _sdf->GetElement("noise")->Get<string>("type").compare("tuw_advanced") == 0 ) {
		foundSimDescription = true;
		sdf::ElementPtr noiseSdf = _sdf->GetElement("noise");
		if( noiseSdf->HasElement("meanWhiteNoise"  ) ) { meanWn_    = fabs( noiseSdf->Get<double>("meanWhiteNoise"  ) ); }
		if( noiseSdf->HasElement("sigmaWhiteNoise" ) ) { sigWn_     = fabs( noiseSdf->Get<double>("sigmaWhiteNoise" ) ); }
		if( noiseSdf->HasElement("tauCutOff"       ) ) { tauBw_     = fabs( noiseSdf->Get<double>("tauCutOff"       ) ); }
		if( noiseSdf->HasElement("sigmaRandWalk"   ) ) { sigRw_     = fabs( noiseSdf->Get<double>("sigmaRandWalk"   ) ); }
		if( noiseSdf->HasElement("sigmaGaussMarkov") ) { sigGm_     = fabs( noiseSdf->Get<double>("sigmaGaussMarkov") ); }
		if( noiseSdf->HasElement("freqGaussMarkov" ) ) { fGm_       = fabs( noiseSdf->Get<double>("freqGaussMarkov" ) ); }
		if( noiseSdf->HasElement("precision"       ) ) { precision_ = fabs( noiseSdf->Get<double>("precision"       ) ); quantized_ = true; }
		if( precision_ < FLT_MIN ) { quantized_ = false; }
	    }
	}
    }
    randGaussWn_ = normal_distribution<double>( meanWn_, sigWn_ );
    return foundSimDescription;
}


