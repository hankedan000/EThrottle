Series off oscope plots showing how I'm timing the ADC measurement of the MC33887's current feedback pin. I'm timing the ADC measurement via the timer0 output compare interrupt service routine (ISR).

CH1 (yellow): motor output (0 to 12v)
CH2 (red): driver feedback pin
CH3 (blue): strobe showing when ADC conversion for feedback read is completed. the ADC measurement is initiated on falling edge of motor output via output compare ISR.