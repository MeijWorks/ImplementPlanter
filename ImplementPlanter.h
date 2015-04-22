/*
  ImplementPlanter - a library for a planter
 Copyright (C) 2011-2014 J.A. Woltjer.
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

#ifndef ImplementPlanter_h
#define ImplementPlanter_h

#include <Arduino.h>
#include <EEPROM.h>
#include "ConfigImplementPlanter.h"
#include "VehicleGps.h"
#include "VehicleTractor.h"

// software version of this library
#define PLANTER_VERSION 0.1

class ImplementPlanter {
private:
  //-------------
  // data members
  //-------------
  
  // Default offset calibration set
  int position_calibration_data[3];
  int position_calibration_points[3];
  int position;
  int last_position;

  // Default xte calibration set  
  int xte_calibration_data[3];
  int xte_calibration_points[3];
  int xte;
  
  int speed;

  // Update timer
  unsigned long update_age;
  boolean update_flag;

  // Variables concerning adjust loop
  byte mode;
  int setpoint;
  int offset;
  byte man_pwm;
  byte auto_pwm;

  int xte_hist[50];
  int xte_sum;      //Running sum of xte_hist
  int xte_avg;      //Average of sum

  int dxte;         //DXTE

  byte hist_count;   //Counter of sum
  byte hist_time;   //Integration time (seconds * 5)

  // PID variables
  float P;
  byte KP;

  float I;
  byte KI;

  float D;
  byte KD;

  // Timers for end shutoff
  int shutoff_time;
  boolean shutoff_wide;
  boolean shutoff_narrow;
  unsigned long shutoff_timer;
  
  // Objects
#ifdef GPS
  VehicleGps * gps;
#else
  VehicleTractor * tractor;
#endif

  //-------------------------------------------------------------
  // private member functions implemented in ImplementPlanter.cpp
  //-------------------------------------------------------------
  int getActualXte();
  int getActualPosition();

  void setSetpoint();

  void readOffset();
  boolean readCalibrationData();
  void printCalibrationData();
  void writeCalibrationData();
  void wipeCalibrationData();

public:
  // -----------------------------------------------------------
  // public member functions implemented in ImplementPlanter.cpp
  // -----------------------------------------------------------

  // Constructor
#ifdef GPS
  ImplementPlanter(VehicleGps * _gps);
#else
  ImplementPlanter(VehicleTractor * _tractor);
#endif

  void update(byte _mode);
  void adjust(int _direction);
  void stop();
  void calibrate();

  // ----------------------------------------------------------------
  // public inline member functions implemented in ImplementPlanter.h
  // ----------------------------------------------------------------
  inline boolean resetCalibration(){
    return readCalibrationData();
  }
  
  inline void commitCalibration(){
    //wipeCalibrationData();
    writeCalibrationData();
  }
  
  // -------
  // Getters
  // -------
  inline boolean getPlantingelement(){
    return digitalRead(PLANTINGELEMENT_PIN);
  }
  
  inline int getPosition(){
    return position;
  }
  
  inline int getXte(){
    return xte;
  }
  
  inline int getSetpoint(){
    return setpoint;
  }
    
  inline int getPositionCalibrationPoint(int _i){
    return position_calibration_points[_i];
  }

  inline int getXteCalibrationPoint(int _i){
    return xte_calibration_points[_i];
  }

  inline byte getKP(){
    return KP;
  }

  inline byte getKI(){
    return KI;
  }

  inline byte getKD(){
    return KD;
  }

  inline byte getPwmMan(){
    return man_pwm;
  }

  inline byte getPwmAuto(){
    return auto_pwm;
  }

  inline int getOffset(){
    return offset;
  }

  // -------
  // Setters
  // -------  
  inline void setPositionCalibrationData(int _i){
    position_calibration_data[_i] = analogRead(POSITION_SENS_PIN);
  }

  inline void setXteCalibrationData(int _i){
    xte_calibration_data[_i] = analogRead(XTE_SENS_PIN);
  }

  inline void setKP(byte _value){
    KP = _value;
  }

  inline void setKI(byte _value){
    KI = _value;
  }

  inline void setKD(byte _value){
    KD = _value;
  }

  inline void setPwmMan(byte _value){
    man_pwm = _value;
  }

  inline void setPwmAuto(byte _value){
    auto_pwm = _value;
  }

  inline void setOffset(int _value){
    offset = _value;
  }
};
#endif