#ifndef LED_H
#define LED_H

class Led{
  private:
		bool _ledState;
		int _pin;
    
  public:
		Led();
		~Led();
		
		void begin(int);		

		bool getState();
		void setState(bool);
};

#endif