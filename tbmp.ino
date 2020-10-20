/*
    Project     'Stream Cheap' Mini Macro Keyboard
    @author     David Madison
    @link       partsnotincluded.com/electronics/diy-stream-deck-mini-macro-keyboard
    @license    MIT - Copyright (c) 2018 David Madison

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.

*/

#include <ClickEncoder.h>
#include <TimerOne.h>
#include <HID-Project.h>


#define ENCODER_CLK A0 // Change A0 to, for example, A5 if you want to use analog pin 5 instead
#define ENCODER_DT A1
#define ENCODER_SW A2

#define KEY_NO_ACTION " "

ClickEncoder *encoder; // variable representing the rotary encoder
int16_t last, value; // variables for current and last rotation value
unsigned int number_of_selected_function = 0; //default to 0

const uint8_t rgbLedPin = 10;
const uint8_t rgbLedValue = 255;

void timerIsr() {
  encoder->service();
}

const uint8_t activeModes = 2;

// Variables will change:
int modePushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

const int ModeButton = 2;    // Set as mode switching button the first button
const int Mode1 = A2;
const int Mode2 = A3; //Mode status LEDs


// Button helper class for handling press/release and debouncing
class button {
  public:
    const char key;
    const char key2;
    const uint8_t pin;

    // Constructor with input pins only (for empty shortcuts).
    button(uint8_t p) : pin(p) {}
    // Constructor with input pin and one key.
    button(uint8_t p, uint8_t k) : pin(p), key(k) {}
    // Constructor with input pin and two keys.
    button(uint8_t p, uint8_t k, uint8_t k2) : pin(p), key(k), key2(k2) {}

    void press(boolean state) {
      if (state == pressed || (millis() - lastPressed  <= debounceTime)) {
        return; // Nothing to see here, folks
      }

      lastPressed = millis();

      if (state == true) {
        Keyboard.press(KeyboardKeycode(key));
        Keyboard.press(key2);
      } else {
        Keyboard.release(KeyboardKeycode(key));
        Keyboard.release(key2);
      }

      pressed = state;
    }

    void update() {
      press(!digitalRead(pin));
    }

  private:
    const long debounceTime = 30;
    unsigned long lastPressed;
    boolean pressed = 0;
} ;

// Initialize button pins
// Button objects, organized in array
button buttons[] = {
  {2}, //l1
  {3}, //l2
  {4}, //l3
  {5}, //l4
  {6}, //l5
  {7}, //r1
  {8}, //r2
  {9}, //r3
  {14}, //r4
  {15}, //r5
};

// Mode 1
// Assign keys to buttons
// Button objects, organized in array
button buttonsMode1[] = {
  {6, KEY_ESC},
  {5, KEY_TAB},
  {4, KEY_LEFT_CTRL, 'c'},
  {3, KEY_LEFT_CTRL, 'v'},
  {2, KEY_LEFT_CTRL},  // Used to be emoty and the swithing pin/button
  {15, KEY_BACKSPACE},
  {14, KEY_LEFT_CTRL, 'y'},
  {9, KEY_LEFT_CTRL, 'z'},
  {8, KEY_ENTER},
  {7, KEY_NO_ACTION, ' '},
};

// Mode 2
// Assign keys to buttons
// Button objects, organized in array
button buttonsMode2[] = {
  {2},  // Empty because it is the swithing pin/button
  {3, KEY_BACKSPACE},  
  {4, KEY_LEFT_CTRL, 'z'},
  {5, KEY_LEFT_CTRL, 'y'},
  {6, KEY_LEFT_CTRL, 's'},
  {7, KEY_ENTER},
  {8, KEY_LEFT_CTRL, 'x'},
  {9, KEY_LEFT_CTRL, 'c'},
  {14, KEY_LEFT_CTRL, 'v'},
  {15, KEY_NO_ACTION, ' '},
};
                    
const uint8_t NumButtons = sizeof(buttons) / sizeof(button);
const uint8_t RXLEDpin = 17; //The RX LED has a defined Arduino pin
const uint8_t TXLEDpin = 30; //The TX LED has a defined Arduino pin

void setup() {
  Serial.begin(9600); // Opens the serial connection used for communication with the PC.
  Consumer.begin(); // Initializes the media keyboard
  encoder = new ClickEncoder(ENCODER_DT, ENCODER_CLK, ENCODER_SW); // Initializes the rotary encoder with the mentioned pins

  Timer1.initialize(1000); // Initializes the timer, which the rotary encoder uses to detect rotation
  Timer1.attachInterrupt(timerIsr);
  last = -1;


  // Safety check. Ground pin #1 (RX) to cancel keyboard inputs.
  pinMode(1, INPUT_PULLUP);
  if (!digitalRead(1)) {
    failsafe();
  }

  // Set LEDs Off. Active low.
  pinMode(RXLEDpin, OUTPUT);
  digitalWrite(RXLEDpin, HIGH);
  TXLED0;

  // Initialize all pins.
  for (int i = 0; i < NumButtons; i++) {
    pinMode(buttons[i].pin, INPUT_PULLUP);
  }
}


void loop() {

  // Blink lights on mode change
  if (digitalRead(2) == LOW) {
    analogWrite(rgbLedPin, 0);
  } else {
    analogWrite(rgbLedPin, rgbLedValue);
  }

  // Disable mode selection (first mode will be used by default)
  //checkModeButton(); // Check which mode is active.

  switch (modePushCounter) { // switch between keyboard configurations:

    // ----------------------------
    // MODE 1
    // ----------------------------
    case 0:    //  Application Alpha or MODE 0

      for (int i = 0; i < NumButtons; i++) {
        buttonsMode1[i].update();
      }
      break;


    // ----------------------------
    // MODE 2
    // ----------------------------
    case 1:    //  Application Beta or MODE 1

      for (int i = 0; i < NumButtons; i++) {
        buttonsMode2[i].update();
      }
      break;

  };



  if (number_of_selected_function == 0) {
    number_of_selected_function = 1;
  };

  if (number_of_selected_function > 4) {
    number_of_selected_function = 1;
  };

  value += encoder->getValue();

  // This part of the code is responsible for the actions when you rotate the encoder
  if (value != last) { // New value is different than the last one, that means to encoder was rotated


    if (number_of_selected_function == 1) {
      if (last < value) // Detecting the direction of rotation
        Consumer.write(MEDIA_VOLUME_DOWN); // Replace this line to have a different function when rotating counter-clockwise
      else
        Consumer.write(MEDIA_VOLUME_UP); // Replace this line to have a different function when rotating clockwise
      last = value; // Refreshing the "last" varible for the next loop with the current value
      Serial.print("Encoder Value: "); // Text output of the rotation value used manily for debugging (open Tools - Serial Monitor to see it)
      Serial.println(value);
    }

    if (number_of_selected_function == 2) {
      if (last < value) { // Detecting the direction of rotation
        delay(50);
        Mouse.move(0, 0, 1);
      }
      else {
        delay(50);
        Mouse.move(0, 0, -1);
      }
      last = value; // Refreshing the "last" varible for the next loop with the current value
      Serial.print("Encoder Value: "); // Text output of the rotation value used manily for debugging (open Tools - Serial Monitor to see it)
      Serial.println(value);
    }

    if (number_of_selected_function == 3) {
      if (last < value) { // Detecting the direction of rotation
        Keyboard.press(KEY_UP_ARROW);
        Serial.print("key: "); // Text output of the rotation value used manily for debugging (open Tools - Serial Monitor to see it)
        Serial.println(KEY_UP_ARROW);
        delay(50);
        Keyboard.releaseAll();
      }
      else {
        Keyboard.press(KEY_DOWN_ARROW);
        delay(50);
        Keyboard.releaseAll();
      }
      last = value; // Refreshing the "last" varible for the next loop with the current value
      Serial.print("Encoder Value: "); // Text output of the rotation value used manily for debugging (open Tools - Serial Monitor to see it)
      Serial.println(value);
    };

    if (number_of_selected_function == 4) {
      if (last < value) { // Detecting the direction of rotation
        Keyboard.press(KEY_LEFT_CTRL);
        delay(50);
        Keyboard.write('-');
        Keyboard.releaseAll();
      }
      else {
        Keyboard.press(KEY_LEFT_CTRL);
        delay(50);
        Keyboard.write('=');
        Keyboard.releaseAll();
      }
      last = value; // Refreshing the "last" varible for the next loop with the current value
      Serial.print("Encoder Value: "); // Text output of the rotation value used manily for debugging (open Tools - Serial Monitor to see it)
      Serial.println(value);
    };

  };

  // This next part handles the rotary encoder BUTTON
  ClickEncoder::Button b = encoder->getButton(); // Asking the button for it's current state
  if (b != ClickEncoder::Open) { // If the button is unpressed, we'll skip to the end of this if block
    //Serial.print("Button: ");
    //#define VERBOSECASE(label) case label: Serial.println(#label); break;
    switch (b) {
      case ClickEncoder::Clicked: // Button was clicked once
        number_of_selected_function++;
        Serial.print("Button pressed: "); // Text output of the rotation value used manily for debugging (open Tools - Serial Monitor to see it)
        Serial.println(number_of_selected_function);
        break;

      case ClickEncoder::DoubleClicked: // Button was double clicked
        number_of_selected_function--;
        Serial.print("Button pressed: "); // Text output of the rotation value used manily for debugging (open Tools - Serial Monitor to see it)
        Serial.println(number_of_selected_function);
        break;
    }
  };

  delay(10); // Wait 10 milliseconds, we definitely don't need to detect the rotary encoder any faster than that
  // The end of the loop() function, it will start again from the beggining (the begginging of the loop function, not the whole document)



}


void checkModeButton() {
  buttonState = digitalRead(ModeButton);
  if (buttonState != lastButtonState) { // compare the buttonState to its previous state
    if (buttonState == LOW) { // if the state has changed, increment the counter
      // if the current state is LOW then the button cycled:
      modePushCounter++;
      //Serial.println("pressed");
      //Serial.print("number of button pushes: ");
      Serial.print("Active Mode: ");
      Serial.println(modePushCounter);

      // Led indication switcher begin
      if (modePushCounter == 1) {
        TXLED1;
        RXLED0;
      } else {
        TXLED0;
        RXLED1;
      }
      // Led indication switcher end
    }
    delay(50); // Delay a little bit to avoid bouncing
  }
  lastButtonState = buttonState; // save the current state as the last state, for next time through the loop
  if (modePushCounter > activeModes - 1) { //reset the counter after 4 presses (remember we start counting at 0)
    modePushCounter = 0;
  }
}

void failsafe() {
  for (;;) {} // Just going to hang out here for awhile :D
}
