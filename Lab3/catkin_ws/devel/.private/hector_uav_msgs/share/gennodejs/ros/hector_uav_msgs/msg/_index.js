
"use strict";

let RC = require('./RC.js');
let Supply = require('./Supply.js');
let Compass = require('./Compass.js');
let YawrateCommand = require('./YawrateCommand.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let RawImu = require('./RawImu.js');
let Altimeter = require('./Altimeter.js');
let MotorPWM = require('./MotorPWM.js');
let ThrustCommand = require('./ThrustCommand.js');
let MotorCommand = require('./MotorCommand.js');
let ControllerState = require('./ControllerState.js');
let MotorStatus = require('./MotorStatus.js');
let HeadingCommand = require('./HeadingCommand.js');
let RawRC = require('./RawRC.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let ServoCommand = require('./ServoCommand.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let RawMagnetic = require('./RawMagnetic.js');
let RuddersCommand = require('./RuddersCommand.js');
let HeightCommand = require('./HeightCommand.js');
let PositionXYCommand = require('./PositionXYCommand.js');

module.exports = {
  RC: RC,
  Supply: Supply,
  Compass: Compass,
  YawrateCommand: YawrateCommand,
  AttitudeCommand: AttitudeCommand,
  RawImu: RawImu,
  Altimeter: Altimeter,
  MotorPWM: MotorPWM,
  ThrustCommand: ThrustCommand,
  MotorCommand: MotorCommand,
  ControllerState: ControllerState,
  MotorStatus: MotorStatus,
  HeadingCommand: HeadingCommand,
  RawRC: RawRC,
  VelocityZCommand: VelocityZCommand,
  ServoCommand: ServoCommand,
  VelocityXYCommand: VelocityXYCommand,
  RawMagnetic: RawMagnetic,
  RuddersCommand: RuddersCommand,
  HeightCommand: HeightCommand,
  PositionXYCommand: PositionXYCommand,
};
