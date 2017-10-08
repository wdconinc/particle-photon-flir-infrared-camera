# particle-photon-flir-infrared-camera
Building a Wifi Infrared Camera by using Particle Photon

# Install:

### Particle Photon: 

Install the `particle-cli` tools using `npm install particle`.

Flash the Particle Photon in the *ArduinoWifiInfraredCamera* folder using
  ```particle compile photon .
     particle flash flirduino FlirDuino.bin```

### Web Application: 
Do following steps Under *WifiInfraredCamera* 

  - Install [node.js]
  - Run
  
  ```sh
  npm install 
  ```
  - Run
  
  ```sh
  npm start
  ```
  - Open [localhost:3000] 

# Note

You should edit following files and put your corresponding server IP and device information
  - *ArduinoWifiInfraredCamera/FlirDuino.ino*
  - *WifiInfraredCamera/public/js/app.js*

[node.js]:http://nodejs.org
[localhost:3000]:http://localhost:3000/

