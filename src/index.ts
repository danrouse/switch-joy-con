import JoyCon from "./joy-con";

/*
examples
https://github.com/charles-toller/joycon/blob/master/Joycon.js
https://github.com/Looking-Glass/JoyconLib/blob/master/Packages/com.lookingglass.joyconlib/JoyconLib_scripts/Joycon.cs
https://github.com/fossephate/JoyCon-Driver/blob/master/joycon-driver/include/Joycon.hpp
https://github.com/fossephate/JoyCon-Driver/blob/master/joycon-driver/src/main.cpp
https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/imu_sensor_notes.md
*/

const joycons = JoyCon.getAllJoyCons();
// console.log('found', joycons.length, 'joycons');
// console.log(joycons);

// process.on('SIGINT', () => {
//   console.log('close');
//   joycons.forEach(j => j.close());
// });


import Joycon, { DebugType } from './joycon_from_cs';

joycons.forEach(d => {
  const joycon = new Joycon(d, true, false, 0);
  joycon.debugType = DebugType.IMU;
  joycon.attach();
  joycon.beginPolling();
});
