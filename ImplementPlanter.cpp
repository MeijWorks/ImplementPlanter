/*
  ImplementPlanter - a libary for a planter
 Copyright (C) 2011-2015 J.A. Woltjer.
 All rights reserved.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ImplementPlanter.h"

//------------
// Constructor
//------------
#ifdef GPS
ImplementPlanter::ImplementPlanter(VehicleGps * _gps){
  // Pin configuration
  // Inputs
  pinMode(PLANTINGELEMENT_PIN, INPUT);
  
  // Outputs
  pinMode(OUTPUT_WIDE, OUTPUT);
  pinMode(OUTPUT_NARROW, OUTPUT);
  pinMode(OUTPUT_BYPASS, OUTPUT);
  pinMode(OUTPUT_LED, OUTPUT);

  // Analog IO
  analogReference(DEFAULT);
  pinMode(POSITION_SENS_PIN, INPUT);
  digitalWrite(POSITION_SENS_PIN, LOW);

  // Get calibration data from EEPROM otherwise use defaults
  if (!readCalibrationData()){
    // Default offset calibration set 100, 101
    position_calibration_data[0] = 201;
    position_calibration_data[1] = 428;
    position_calibration_data[2] = 687;

    // Default xte calibration set 110, 111
    xte_calibration_data[0] = 201;
    xte_calibration_data[1] = 428;
    xte_calibration_data[2] = 687;

    // PID constants 120, 121, 130, 131, 140, 141
    KP = 50;
    KI = 50;
    KD = 1;

    // pwm values for manual (150) and auto (160)
    man_pwm = 90;
    auto_pwm = 70;

    // offset 180
    offset = 0;
    
#ifdef DEBUG
    Serial.println("No calibration data found");
#endif
  }

  // Calibration points for position and xte
  position_calibration_points[0] = -6;
  position_calibration_points[1] = 0;
  position_calibration_points[2] = 6;
  position = 0;
  last_position = 0;

  xte_calibration_points[0] = -10;
  xte_calibration_points[1] =  0;
  xte_calibration_points[2] =  10;
  xte = 0;

  // Setpoint of ajust loop
  setpoint = 0;

  // PID integration and differentiation intervals
  for (int i = 0; i < 50; i++){
    xte_hist[i] = 0;
  }
  xte_sum = 0;      //Running sum of xte_hist
  xte_avg = 0;      //Average of sum

  dxte = 0;         //DXTE

  hist_count = 0;   //Counter of sum
  hist_time = 25;   //Integration time (seconds * 5) == 5 in this case

  // PID variables
  P = 0;
  I = 0;
  D = 0;

  // End shutoff timers
  shutoff_time = SHUTOFF;
  shutoff_wide = false;
  shutoff_narrow = false;
  shutoff_timer = millis();

  // Update timer
  update_age = millis();
  
  // Print calibration data
#ifdef DEBUG
  printCalibrationData();
#endif

  // Connected classes
#ifdef GPS
  gps = _gps;
#else
  tractor = _tractor;
#endif
}
#else
ImplementPlanter::ImplementPlanter(VehicleTractor * _tractor){
  // Pin configuration
  // Inputs
  pinMode(PLANTINGELEMENT_PIN, INPUT);
  
  // Outputs
  pinMode(OUTPUT_WIDE, OUTPUT);
  pinMode(OUTPUT_NARROW, OUTPUT);
  pinMode(OUTPUT_BYPASS, OUTPUT);
  pinMode(OUTPUT_LED, OUTPUT);

  // Analog IO
  analogReference(DEFAULT);
  pinMode(POSITION_SENS_PIN, INPUT);
  digitalWrite(POSITION_SENS_PIN, LOW);

  pinMode(XTE_SENS_PIN, INPUT);
  digitalWrite(XTE_SENS_PIN, LOW);

  // Get calibration data from EEPROM otherwise use defaults
  if (!readCalibrationData()){
    // Default offset calibration set 100, 101
    position_calibration_data[0] = 201;
    position_calibration_data[1] = 428;
    position_calibration_data[2] = 687;

    // Default xte calibration set 110, 111
    xte_calibration_data[0] = 201;
    xte_calibration_data[1] = 428;
    xte_calibration_data[2] = 687;

    // PID constants 120, 121, 130, 131, 140, 141
    KP = 50;
    KI = 50;
    KD = 1;

    // pwm values for manual (150) and auto (160)
    man_pwm = 90;
    auto_pwm = 70;

    // offset 180
    offset = 0;
    
#ifdef DEBUG
    Serial.println("No calibration data found");
#endif
  }

  // Calibration points for position and xte
  position_calibration_points[0] = -6;
  position_calibration_points[1] = 0;
  position_calibration_points[2] = 6;
  position = 0;
  last_position = 0;

  xte_calibration_points[0] = -10;
  xte_calibration_points[1] =  0;
  xte_calibration_points[2] =  10;
  xte = 0;

  // Setpoint of ajust loop
  setpoint = 0;
  
  // Speed
  speed = 0;

  // PID integration and differentiation intervals
  for (int i = 0; i < 50; i++){
    xte_hist[i] = 0;
  }
  xte_sum = 0;      //Running sum of xte_hist
  xte_avg = 0;      //Average of sum

  dxte = 0;         //DXTE

  hist_count = 0;   //Counter of sum
  hist_time = 25;   //Integration time (seconds * 5)

  // PID variables
  P = 0;
  I = 0;
  D = 0;

  // End shutoff timers
  shutoff_time = SHUTOFF;
  shutoff_wide = false;
  shutoff_narrow = false;
  shutoff_timer = millis();

  // Update timer
  update_age = millis();
  
  // Print calibration data
#ifdef DEBUG
  printCalibrationData();
#endif
  tractor = _tractor;
}
#endif

// ----------------------------------
// Method for updating implement data
// ----------------------------------
void ImplementPlanter::update(byte _mode){
  mode = _mode;
#ifdef GPS
  // update offset, xte, position and setpoint
  if (millis() - update_age >= 200){
    // Update xte, position and setpoint
    xte = gps->getXte();
    speed = gps->getSpeedMs();
    position = getActualPosition();

    setSetpoint();
    update_age = millis();    
  }
#else
  // update offset, xte, position and setpoint
  if (millis() - update_age >= 25){
    // Update xte, position and setpoint
    if (update_flag){
      update_flag = false;
      
      // read xte, reset ad converter to position input
      xte = getActualXte();
      speed = tractor->getSpeedMs();
      getActualPosition();
    }
    else {
      update_flag = true;
      
      // read position, reset ad converter to xte input
      position = getActualPosition();
      getActualXte();
    }

    setSetpoint();
    update_age = millis();    
  }
#endif
}

// ------------------------------
// Method for adjusting implement
// ------------------------------
void ImplementPlanter::adjust(int _direction){
  int _actual_position;
  byte _pwm;

  if (mode == 0){
    _actual_position = position;
    _pwm = auto_pwm;
#ifdef RELAY
    digitalWrite(OUTPUT_BYPASS, HIGH);
#endif  
  }
  else {
    _actual_position = setpoint - _direction;
    _pwm = man_pwm;
#ifdef RELAY
    if (_actual_position != setpoint) {
      digitalWrite(OUTPUT_BYPASS, HIGH);
    }
    else {
      digitalWrite(OUTPUT_BYPASS, LOW);
    }
#endif
  }

  // Adjust tree including error
  //---------------------------
  // Setpoint < actual position
  //---------------------------
  if (_actual_position < setpoint && !shutoff_narrow){
#ifndef FACTORY
  //TODO shutoff < 0
    digitalWrite(OUTPUT_WIDE, LOW);
    digitalWrite(OUTPUT_NARROW, HIGH);
    analogWrite(OUTPUT_BYPASS, _pwm);
#else
    digitalWrite(OUTPUT_WIDE, LOW);
    analogWrite(OUTPUT_NARROW, _pwm);
#endif
    digitalWrite(OUTPUT_LED, HIGH);
    
    // End shutoff
    if (_actual_position != last_position){
      shutoff_timer = millis();
    }
    
    if (shutoff_wide){
      shutoff_narrow = false;
      shutoff_wide = false;
    }
    
    if (millis() - shutoff_timer > shutoff_time){
      shutoff_narrow = true;
      shutoff_wide = false;
    }
  }
  //---------------------------
  // Setpoint < actual position
  //---------------------------
  else if (_actual_position > setpoint && !shutoff_wide){
#ifndef FACTORY
  //TODO shutoff > 0
    digitalWrite(OUTPUT_WIDE, HIGH);
    digitalWrite(OUTPUT_NARROW, LOW);
    analogWrite(OUTPUT_BYPASS, _pwm);
#else
    analogWrite(OUTPUT_WIDE, _pwm);
    digitalWrite(OUTPUT_NARROW, LOW);
#endif
    digitalWrite(OUTPUT_LED, HIGH);

    // End shutoff
    if (_actual_position != last_position){
      shutoff_timer = millis();
    }
    
    if (shutoff_narrow){
      shutoff_wide = false;
      shutoff_narrow = false;
    }
    
    if (millis() - shutoff_timer > shutoff_time){
      shutoff_narrow = false;
      shutoff_wide = true;
    }
  }
  //-----------------
  // Setpoint reached
  //-----------------
  else {
    digitalWrite(OUTPUT_WIDE, LOW);
    digitalWrite(OUTPUT_NARROW, LOW);
#ifndef RELAY
    digitalWrite(OUTPUT_BYPASS, LOW);
#endif
    digitalWrite(OUTPUT_LED, LOW);
    shutoff_timer = millis();
  }
  last_position = _actual_position;
}

// ------------------------------
// Method for stopping implement
// ------------------------------
void ImplementPlanter::stop(){
  digitalWrite(OUTPUT_WIDE, LOW);
  digitalWrite(OUTPUT_NARROW, LOW);
#ifndef RELAY
  digitalWrite(OUTPUT_BYPASS, LOW);
#endif
  digitalWrite(OUTPUT_LED, LOW);
}

// --------------------------------------------
// Method for measuring actual implement offset
// --------------------------------------------
int ImplementPlanter::getActualPosition(){
  // Read analog input
  int _read_raw = analogRead(POSITION_SENS_PIN);
  int _actual_position;
  int i = 0;

  // Loop through calibrationdata
  while (_read_raw > position_calibration_data[i] && i < 2){
    i++;
  }

  if (i == 0){
    i++;
  }

  // Interpolate calibrationdata
  float a = _read_raw - position_calibration_data[i-1];
  float b = position_calibration_data[i] - position_calibration_data[i-1];
  float c = position_calibration_points[i] - position_calibration_points[i-1];
  float d = position_calibration_points[i-1];

  // Calculate actual implement offset
  _actual_position = (((a * c) / b) + d);

  return _actual_position;
}

//------------------------------------------
// Method for measuring actual implement XTE
//------------------------------------------
int ImplementPlanter::getActualXte(){
  // Read analog input
  int _read_raw = analogRead(XTE_SENS_PIN);
  int _actual_xte;
  int i = 0;

  // Loop through calibrationdata
  while (_read_raw > xte_calibration_data[i] && i < 2){
    i++;
  }

  if (i == 0){
    i++;
  }

  // Interpolate calibrationdata
  float a = _read_raw - xte_calibration_data[i-1];
  float b = xte_calibration_data[i] - xte_calibration_data[i-1];
  float c = xte_calibration_points[i] - xte_calibration_points[i-1];
  float d = xte_calibration_points[i-1];

  // Calculate actual implement offset
  _actual_xte = (((a * c) / b) + d);

  return _actual_xte;
}

// ---------------------------
// Method for setting setpoint
// ---------------------------
void ImplementPlanter::setSetpoint(){
  int _previous_hist_count;
  int _xte_avg;
  int _dxte;
  int _dxte_calc;
  int _D_factor;
  int _xte_cor = xte + offset;

  _previous_hist_count = hist_count - 1;

  if (hist_count >= hist_time) {
    hist_count = 0;
    _previous_hist_count = hist_time - 1;
  }

  // Set xte sum , delta and averages;
  xte_sum = xte_sum - xte_hist[hist_count] + _xte_cor;
  _xte_avg = xte_sum / hist_time;
  _dxte = _xte_cor - xte_hist[_previous_hist_count];
  _dxte_calc = _xte_cor;//sqrt(abs(_xte))
  _D_factor = _dxte - _dxte_calc;

  // Update XTE history
  xte_hist[hist_count] = _xte_cor;

  P = float(_xte_cor) * KP / 100.0f; 
  I = float(_xte_avg) * KI / 100.0f;
  D = D - (float(_D_factor) * KD / 100.0f) * speed;
  
  // Restrict D
  if (D > 15) {
    D = 15;
  }
  else if (D < -15) {
    D = -15;
  }

  // Reset D
  if (mode){
    D = 0;
  }

  setpoint = P + I + D;   

  hist_count++; 
}

// ----------------------------------------------
// Method for reading calibrationdata from EEPROM
// ----------------------------------------------
boolean ImplementPlanter::readCalibrationData(){
  // Read amount of startups and add 1
  EEPROM.write(0, EEPROM.read(0) + 1);
  
  // Read offset and XTE calibration data
  if (EEPROM.read(100) != 255 || EEPROM.read(110) != 255 ||
    EEPROM.read(120) != 255 || EEPROM.read(130) != 255 ||
    EEPROM.read(140) != 255 || EEPROM.read(150) != 255 ||
    EEPROM.read(160) != 255 || EEPROM.read(180) != 255){

    // Read from eeprom highbyte, then lowbyte, and combine into words
    for(int i = 0; i < 3; i++){
      int k = 2 * i;
      // 100 - 101 and 110 - 111
      position_calibration_data[i] = word(EEPROM.read(k+100), EEPROM.read(k+101));
      xte_calibration_data[i] = word(EEPROM.read(k+110), EEPROM.read(k+111));
    }

    KP = EEPROM.read(120);
    KI = EEPROM.read(130);
    KD = EEPROM.read(140);

    man_pwm = EEPROM.read(150);
    auto_pwm = EEPROM.read(160);
    
    if (EEPROM.read(180) < 255 || EEPROM.read(181) < 255){
      // Read offset (2 bytes)
      offset = word(EEPROM.read(180), EEPROM.read(181));
      if (offset > 20 || offset < -20){
        offset = 0;
      }
    }
    else {
      offset = 0;
    }
  }
  else {
    return false;
  }
  return true;
}

//---------------------------------------------------
//Method for printing calibration data to serial port
//---------------------------------------------------
void ImplementPlanter::printCalibrationData(){
  // Print amount of times started
  Serial.println("Times started");
  Serial.println(EEPROM.read(0));
  Serial.println("-------------------------------");
  
  // Printing calibration data to serial port
  Serial.println("Using following data:");
  Serial.println("-------------------------------");
  Serial.println("Offset calibration data");
  for (int i = 0; i < 3; i++){
    Serial.print(position_calibration_data[i]);
    Serial.print(", ");
    Serial.println(position_calibration_points[i]);
  }
  Serial.println("-------------------------------");

#ifndef GPS
  Serial.println("XTE calibration data");
  for (int i = 0; i < 3; i++){
    Serial.print(xte_calibration_data[i]);
    Serial.print(", ");
    Serial.println(xte_calibration_points[i]);
  }
  Serial.println("-------------------------------");
#endif
  
  Serial.println("KP");
  Serial.println(KP);
  Serial.println("-------------------------------");

  Serial.println("KI");
  Serial.println(KI);
  Serial.println("-------------------------------");

  Serial.println("KD");
  Serial.println(KD);
  Serial.println("-------------------------------");
  
  Serial.println("PWM auto");
  Serial.println(auto_pwm);
  Serial.println("-------------------------------");
  
  Serial.println("PWM manual");
  Serial.println(man_pwm);
  Serial.println("-------------------------------");
}

// --------------------------------------------
// Method for writing calibrationdata to EEPROM
// --------------------------------------------
void ImplementPlanter::writeCalibrationData(){
  // Write each byte separately to the memory first the data then the points
  for(int i = 0; i < 3; i++){
    int k = 2 * i;
    EEPROM.write(k+100, highByte(position_calibration_data[i])); // 100, 102, 104
    EEPROM.write(k+101, lowByte(position_calibration_data[i]));  // 101, 103, 105
    EEPROM.write(k+110, highByte(xte_calibration_data[i])); // 110, 112, 114
    EEPROM.write(k+111, lowByte(xte_calibration_data[i]));  // 111, 113, 115
  }

  EEPROM.write(120, KP); // 120
  EEPROM.write(130, KI); // 130
  EEPROM.write(140, KD); // 140
  
  EEPROM.write(150, man_pwm);  //150
  EEPROM.write(160, auto_pwm); //160
  
  EEPROM.write(180, highByte(offset)); // 180
  EEPROM.write(181, lowByte(offset)); // 181

#ifdef DEBUG
  Serial.println("Calibration data written");
#endif  
}

// ---------------------------------------------
// Method for wiping calibrationdata from EEPROM
// ---------------------------------------------
void ImplementPlanter::wipeCalibrationData(){
  // Wipe calibration data

    // Write 255 into all memory registers
  for  (int i = 1; i < 255; i++){
    EEPROM.write(i, 255);
  }

#ifdef DEBUG
  Serial.println("Calibration data wiped");
#endif
}