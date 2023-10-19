//include libraries
#include <avr/io.h>
#include <avr/interrupt.h>


//define global variables
char state = 0;
unsigned int compare = 7812;
unsigned int direction = 0;
unsigned int step = 0;
const unsigned int max_frequency = 3;
const unsigned int steps = 6;
const unsigned int step_theta = 360 / steps;
unsigned int sine[] = {0, 175, 349, 523, 698, 872, 1045, 1219, 1392, 1564, 1736, 1908, 2079, 2250, 2419, 2588, 2756, 2924, 3090, 3256, 3420, 3584, 3746, 3907, 4067, 4226, 4384, 4540, 4695, 4848, 5000, 5150, 5299, 5446, 5592, 5736, 5878, 6018, 6157, 6293, 6428, 6561, 6691, 6820, 6947, 7071, 7193, 7314, 7431, 7547, 7660, 7771, 7880, 7986, 8090, 8192, 8290, 8387, 8480, 8572, 8660};
unsigned int compare_array[steps][3];


//functions
void states(char state);
void initialization();
void red_off(){		PORTD |= 1 << 6;}
void red_on(){		PORTD &= ~(1 << 6);}
void yellow_off(){	PORTB |= 1 << 1;}
void yellow_on(){	PORTB &= ~(1 << 1);}
void green_off(){	PORTB |= 1 << 3;}
void green_on(){	PORTB &= ~(1 << 3);}


//timer interrupt executed when compare value is reached
ISR(TIMER1_COMPA_vect){
	//if direction variable is set to zero, execute right vector part
	if (direction == 0){
		//execute output configuration
		states(state);

		//assign time for right vector component
		//OCR1A = compare_array[step][direction];
		OCR1A = 7812;

		//count up direction variable
		direction++;
	}
	//if direction variable is set to one, execute left vector part
	else if (direction == 1) {
		//execute output configuration, if right vector is vector 6 execute vector 1 as left component
		if (state == 6) {
			states(1);
		}
		else{
			states(state + 1);
		}

		//assign time for left vector component to compare register
		//OCR1A = compare_array[step][direction];
		OCR1A = 7812;

		//count up direction
		direction++;
	}
	//if direction variable is set to two, execute zero vector part
	else if (direction == 2) {
		//execute zero vector configuration
		states(0);

		//assign time for zero vector component to compare register
		//OCR1A = compare_array[step][direction];
		OCR1A = 7812;

		//reset direction and count up step
		direction = 0;
		step++;
	}

	if ((step * step_theta) >= 60) {
		state = 2;
	}
	else if ((step * step_theta) >= 120) {
		state = 3;
	}
	else if ((step * step_theta) >= 180) {
		state = 4;
	}
	else if ((step * step_theta) >= 240) {
		state = 5;
	}
	else if ((step * step_theta) >= 300) {
		state = 6;
	}
	else if ((step * step_theta) >= 360) {
		//when the whole rotation is finished we restart from the beginning with state one and reset step variable
		state = 1;
		step = 0;
	}
}

//interrupt service routine for analog read and compare value calculation
ISR(ADC_vect) {
	//stop timer to prevent Timer Interrupt during calculation
	TCCR1B = 0;

	//compare value calculation (directly proportional to T_v) --> Ticks for Tv / 10000
	compare = 1600 / (max_frequency * steps * (256 - ADCH));

	//define angle variable
	short theta = 0; 

	//calculate every necessary compare value in before
	//loop for every step from 0 degree to less than 360 degree
	for (unsigned int i = 0; i < steps; i++){
		//calculate angle for current step with respect to left vector component (60 degrees between left and right vector)
		theta = (i * step_theta) % 60;

		//loop for every direction in every step
		for (int j = 0; j < 3; j++){
			//if direction loop is equal zero execute calculation for right component
			if (j == 0){
				//T_r calculation --> Ticks for T_r, sine table is * 10000, so with (Ticks for Tv / 10000) it cancels out
				compare_array[i][j] = compare * sine[60 - theta];
			}
			//if direction loop is equal one execute calculation for left component
			else if (j == 1){
				//T_l calculation --> Ticks for T_l, sine table is * 10000, so with (Ticks for Tv / 10000) it cancels out
				compare_array[i][j] = compare * sine[theta];
			}
			//if direction loop is equal two execute calculation for zero component
			else if (j == 2){
				//T_0 calculation --> Ticks for T_0, compare * 10000 to make compare value consistens to how T_r and T_l have been calculated
				compare_array[i][j] = (compare * 10000) - compare_array[i][0] - compare_array[i][1];
			}
		}
	}

	//start timer again
	TCCR1B = 0x0D;
}

//external interrupt on push button 1 to start single ADC conversion
ISR(INT0_vect){
	//start AD conversion
	ADCSRA |= 0x40;
}

int main(void) {
	//initialize system
	initialization();

	while (1) {}
}

//initialization
void initialization(){
	//output configuration
	DDRD |= 1 << 6; //red
	DDRB |= 1 << 1; //yellow
	DDRB |= 1 << 3; //green

	//timer configuration for 16-bit timer
	TCCR1A = 0;
	//configure interrupt flag to interrupt on compare register A
	TIMSK1 = 0x02;
	//set value in compare register to get interrupt after 500ms
	OCR1A = compare;

	//configuring ADC,enable PIN A0 and use internal reference voltage AVcc
	ADMUX |= 0x60;
	//enable ADC, enable interrupts and use prescaler 128 (biggest available)
	ADCSRA = 0xCF;

	//enable External Interrupt
	EICRA |= 0x02;
	//enable External Interrupt Mask for push button 1
	EIMSK |= 0x01;

	//set enable interrupt, enabling interrupts globally
	sei();

	//set initial state
	states(0);

	//start timer with prescaler 1024 and Count-To-Compare configuration to interrupt on compare with OCR1A register 
	TCCR1B = 0x0D;
}

//states function
void states(char state){
	switch (state){
	case 0:
		//all off
		red_off();
		yellow_off();
		green_off();
		break;
	case 1:
		//red on
		red_on();
		yellow_off();
		green_off();
		break;
	case 2:
		//green off
		red_on();
		yellow_on();
		green_off();
		break;
	case 3:
		//yellow on
		red_off();
		yellow_on();
		green_off();
		break;
	case 4:
		//red off
		red_off();
		yellow_on();
		green_on();
		break;
	case 5:
		//green on
		red_off();
		yellow_off();
		green_on();
		break;
	case 6:
		//yellow off
		green_on();
		yellow_off();
		red_on();
		break;
	}
}