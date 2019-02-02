/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef filter_biquad_h_
#define filter_biquad_h_

#include "Arduino.h"
#include "AudioStream.h"

class AudioFilterBiquad : public AudioStream
{
public:
	AudioFilterBiquad(void) : AudioStream(1, inputQueueArray) {
		// by default, the filter will not pass anything
		for (int i=0; i<32; i++) definition[i] = 0;
	}
	virtual void update(void);

	// Set the biquad coefficients directly
	void setCoefficients(uint32_t stage, const int *coefficients);
	void setCoefficients(uint32_t stage, const double *coefficients) {
		int coef[5];
		coef[0] = coefficients[0] * 1073741824.0;
		coef[1] = coefficients[1] * 1073741824.0;
		coef[2] = coefficients[2] * 1073741824.0;
		coef[3] = coefficients[3] * 1073741824.0;
		coef[4] = coefficients[4] * 1073741824.0;
		setCoefficients(stage, coef);
	}

	/*
	  #define FILTER_LOPASS 0
  #define FILTER_HIPASS 1
  #define FILTER_BANDPASS 2
  #define FILTER_NOTCH 3
  #define FILTER_PARAEQ 4
  #define FILTER_LOSHELF 5
  #define FILTER_HISHELF 6
	*/
	
	// Compute common filter functions
	// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
	void setLowpass(uint32_t stage, float frequency, float q = 0.7071) {
		int coef[5];
		calcBiquad(0, frequency, 0, q, coef);
		setCoefficients(stage, coef);
	}
	void setHighpass(uint32_t stage, float frequency, float q = 0.7071) {
		int coef[5];
		calcBiquad(1, frequency, 0, q, coef);
		setCoefficients(stage, coef);
	}
	void setBandpass(uint32_t stage, float frequency, float q = 1.0) {
		int coef[5];
		calcBiquad(2, frequency, 0, q, coef);
		setCoefficients(stage, coef);
	}
	void setNotch(uint32_t stage, float frequency, float q = 1.0) {
		int coef[5];
		calcBiquad(3, frequency, 0, q, coef);
		setCoefficients(stage, coef);
	}
	void setLowShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f) {
		int coef[5];
		calcBiquad(5, frequency, gain, slope, coef);
		setCoefficients(stage, coef);
	}
	void setHighShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f) {
		int coef[5];
		calcBiquad(6, frequency, gain, slope, coef);
		setCoefficients(stage, coef);
	}
	
	static void calcBiquad(uint8_t filtertype, float freq, float gain, float q, int *coef)
	{
		double w0 = freq * (2 * 3.141592654 / AUDIO_SAMPLE_RATE_EXACT);
		double sinW0 = sin(w0);
		double alpha = sinW0 / ((double)q * 2.0);
		double cosW0 = cos(w0);
		double scale = 1073741824.0 / (1.0 + alpha);
		double a = pow(10.0, gain/40.0);
		//generate three helper-values (intermediate results):
		double sinsq = sinW0 * sqrt( (pow(a,2.0)+1.0)*(1.0/q-1.0)+2.0*a );
		double aMinus = (a-1.0)*cosW0;
		double aPlus = (a+1.0)*cosW0;
		double scaleLow = 1073741824.0 / ( (a+1.0) + aMinus + sinsq);
		double scaleHigh = 1073741824.0 / ( (a+1.0) - aMinus + sinsq);
		
		switch (filtertype) {
		case 0:
			/* b0 */ coef[0] = ((1.0 - cosW0) / 2.0) * scale;
			/* b1 */ coef[1] = (1.0 - cosW0) * scale;
			/* b2 */ coef[2] = coef[0];
			/* a1 */ coef[3] = (-2.0 * cosW0) * scale;
			/* a2 */ coef[4] = (1.0 - alpha) * scale;
		break;
		case 1:
			/* b0 */ coef[0] = ((1.0 + cosW0) / 2.0) * scale;
			/* b1 */ coef[1] = -(1.0 + cosW0) * scale;
			/* b2 */ coef[2] = coef[0];
			/* a1 */ coef[3] = (-2.0 * cosW0) * scale;
			/* a2 */ coef[4] = (1.0 - alpha) * scale;
		break;
		case 2:
			/* b0 */ coef[0] = alpha * scale;
			/* b1 */ coef[1] = 0;
			/* b2 */ coef[2] = (-alpha) * scale;
			/* a1 */ coef[3] = (-2.0 * cosW0) * scale;
			/* a2 */ coef[4] = (1.0 - alpha) * scale;
		break;
		case 3:
			/* b0 */ coef[0] = scale;
			/* b1 */ coef[1] = (-2.0 * cosW0) * scale;
			/* b2 */ coef[2] = coef[0];
			/* a1 */ coef[3] = (-2.0 * cosW0) * scale;
			/* a2 */ coef[4] = (1.0 - alpha) * scale;
		break;
		case 5:
			/* b0 */ coef[0] =		a *	( (a+1.0) - aMinus + sinsq	) * scaleLow;
			/* b1 */ coef[1] =  2.0*a * ( (a-1.0) - aPlus  			) * scaleLow;
			/* b2 */ coef[2] =		a * ( (a+1.0) - aMinus - sinsq 	) * scaleLow;
			/* a1 */ coef[3] = -2.0*	( (a-1.0) + aPlus			) * scaleLow;
			/* a2 */ coef[4] =  		( (a+1.0) + aMinus - sinsq	) * scaleLow;
			break;
		case 6:
			/* b0 */ coef[0] =		a *	( (a+1.0) + aMinus + sinsq	) * scaleHigh;
			/* b1 */ coef[1] = -2.0*a * ( (a-1.0) + aPlus  			) * scaleHigh;
			/* b2 */ coef[2] =		a * ( (a+1.0) + aMinus - sinsq 	) * scaleHigh;
			/* a1 */ coef[3] =  2.0*	( (a-1.0) - aPlus			) * scaleHigh;
			/* a2 */ coef[4] =  		( (a+1.0) - aMinus - sinsq	) * scaleHigh;
				break;
		default:
		break;
		}
	}

private:
	int32_t definition[32];  // up to 4 cascaded biquads
	audio_block_t *inputQueueArray[1];
};

#endif
