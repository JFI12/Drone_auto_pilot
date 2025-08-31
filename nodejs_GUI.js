const express = require('express');
const { SerialPort } = require('serialport'); // Ensure correct import
const { ReadlineParser } = require('@serialport/parser-readline'); // Ensure correct import
const app = express();
const port = 3000;

// Replace '/dev/ttyUSB0' with your Arduino serial port path


const arduinoPort = new SerialPort({
    path:'COM3',
    baudRate:115200,
    parser: new ReadlineParser("\n"),
  });
const parser = arduinoPort.pipe(new ReadlineParser({ delimiter: '\n' }));
app.use(express.static('public'));

app.get('/set-brightness/:value', (req, res) => {
  const value = req.params.value;
  if (value >= 900 && value <= 1000) {
    arduinoPort.write(value + '\n');
    res.send(`Brightness set to ${value}`);
  } else {
    res.send('Invalid value');
  }
});

app.listen(port, () => {
  console.log(`Server running at http://localhost:${port}`);
});

parser.on('data', data => {
  console.log('Received from Arduino:', data.trim());
});