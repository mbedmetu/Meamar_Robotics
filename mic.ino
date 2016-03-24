
#include "arduinoFFT.h"

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 47000;

double vReal[samples]; 
double vImag[samples];

void setup()
{
  pinMode(A1, INPUT);
  sbi(ADCSRA, 1);
  cbi(ADCSRA, 1);
  cbi(ADCSRA, 0);
  Serial.begin(9600);
  Serial.println("Ready");
}

void loop() 
{
  
  for (uint8_t i = 0; i < samples; i++) 
  {
    int mic= analogRead(A1);
    vReal[i] = mic;
  }
 
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */	
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  Serial.println(x, 6);
  memset(vReal, 0, sizeof(vReal));
  memset(vImag, 0, sizeof(vImag));
 
}
