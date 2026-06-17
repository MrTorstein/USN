// Imports
#include <Low_pass_filter.h>
#include <TMP36.h>
#include <TNC10.h>

// Module definitions
Low_pass_filter LPF;
TMP36 TMP;
TNC10 TNC;

// Low pass filter variables
float filter_constant = 0.1;
float filtered_tmp_adc = 153;
float filtered_tnc_adc = 445;

void setup() {
  Serial.begin(9600);
}

void loop() {
  //filtered_tmp_adc = LPF.filter(analogRead(A0), filtered_tmp_adc, filter_constant);
  filtered_tnc_adc = LPF.filter(analogRead(A0), filtered_tnc_adc, filter_constant);
  //float temperature = TMP.convertToC(filtered_tmp_adc);
  float temperature = TNC.convertToC(filtered_tnc_adc);
  Serial.print("Temperature: ");
  Serial.println(temperature);

  delay(1000);
}
