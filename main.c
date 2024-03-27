#define F_CPU 1000000UL // Takt der anliegt
// Welche Pins liegen beim ATiny an jeweiligen buttons an
#define BUTTON1			PINA1 // +
#define BUTTON1_PIN		PINA
#define BUTTON2			PINA2  // -
#define BUTTON2_PIN		PINA
#define BUTTON3			PINA0 // PWR/MODI
#define BUTTON3_PIN		PINA

//Zählung für das Debouncen der Knöpfe
#define BUTTON_SAMPLE_FREQUENCY		40 // Knopf soll 40 mal pro Sekunde untersucht werden
#define BUTTON_DEBOUNCE_TIME		0.075	// (in s)
#define MAXIMUM (BUTTON_DEBOUNCE_TIME * BUTTON_SAMPLE_FREQUENCY)
#define freq (45 * 11) // 45 Hz * 11 LED = 495 Hz | von Leds
#define remainder (F_CPU / 64 / freq) // bei 64 Prescale (bei 1MHz 31 timer ticks) Rechner: https://eleccelerator.com/avr-timer-calculator/
#define button_timer (freq / BUTTON_SAMPLE_FREQUENCY) // 10 Hz sind also 49

#define CHECK_BIT(var,pos) ((var >> pos) & 1) // Überprüft ob Bit an einer bestimmten Stelle 1 ist | z.B überprüft durch verschieben ob Button 1 ist im Buttonhandler
#define GET_BITS(var, pos, n) (((var) >> (pos)) & ((1 << n) - 1)) // benötigt um Integrator hochzuzählen

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

struct time { //Speichern der aktuellen Zeit
	uint32_t beforesecond; // 0 - 124
	uint8_t second; // 0-59
	uint8_t minute; // 0-59
	uint8_t hour; // 0-23
};

struct time time; // Erstellen eine Variable time

// Belegung der Dioden - Anspruch des Charlieplexings
const uint8_t M[] =
{
	((PINA4 << 3) | PINA3), // D1 PinA4 wird um 3 Stellen verschoben und mit PINA3 verodert
	((PINA5 << 3) | PINA3), // D2
	((PINA6 << 3) | PINA3), // D3
	((PINA3 << 3) | PINA4), // D4
	((PINA5 << 3) | PINA4), // D5
	((PINA6 << 3) | PINA4), // D6
	((PINA4 << 3) | PINA5), // D7
	((PINA6 << 3) | PINA5), // D8
	((PINA3 << 3) | PINA5), // D9
	((PINA3 << 3) | PINA6), // D10
	((PINA4 << 3) | PINA6) // D11
};
volatile uint16_t screen = 0; // weiß welche Leds an sind
// Zuordnung zwischen Led Pin und Screen Pin
// Screen Bitbelegung: 0 0 D6 D8 D9 D7 D10 D11 0 0 0 D1 D2 D3 D4 D5
// Beispiel: D3 -> 3 Lesen in S von vorne anfangen 4 3 2 -> 2  | Die Zahl von S = Pos (2) in Screen lesen von hinten anfangen D5(0) D4(1) D3(2) -> D3
volatile const uint8_t S[11] = {4, 3, 2, 1, 0, 13, 12, 11, 10, 9, 8}; 
	
volatile uint8_t active_diode = 0; // Welche Diode ist gerade dran beim Charlieplexing

volatile uint8_t button_timer_counter; 
uint8_t buttons_modus = 0;  // Hinterlegt was wirklich gedrückt wurde (0 oder 1)

/*Fürs Debouncen
real signal 0000111111110000000111111100000000011111111110000000000111111100000
corrupted   0100111011011001000011011010001001011100101111000100010111011100010
integrator  0100123233233212100012123232101001012321212333210100010123233321010
output      0000001111111111100000001111100000000111111111110000000001111111000
*/
volatile uint8_t buttons_integrator = 0; // Zählt hoch wenn Button gedrückt wurde


void interrupt_leds_off(void)
{
	// Setze alle LED-Pins auf Input (0) und ohne Pullup-Widerstand (0)
	// DDRA regelt ob Ein oder Ausgang | PORTA regelt wenn man es als Eingang betrachtet, ob es Widerstände gibt
	DDRA &= ~((1 << PINA6) | (1 << PINA5) | (1 << PINA4) | (1 << PINA3));
	PORTA &= ~((1 << PINA6) | (1 << PINA5) | (1 << PINA4) | (1 << PINA3));
}

void timer_init(void) // damit Timer bei 1s auch wirklich 1s hochzählt
{
	// läuft mit 1Mhz
	// 1/1000000 * 64 * 125 * 125 = 1	
	
	TCCR0B |= (1 << CS01) | (1 << CS00); // prescale = 64
	OCR0A = 124; // 124 weil von 0 gez�hlt - soll Interrupt nach draußen senden
	// Einstellungen für Microship: Setze Werte ins Register
	TIMSK |= (1 << OCIE0A); // enable compare interrupt for A in Time Interrupt Mask Register
	TCCR0A |= (1 << 0); // enable CTC (Clear Timer on Compare match) - setzt 124 wieder auf 0
}

void led_init(void)
{	// starten eines weiteren Timers
	interrupt_leds_off();
	TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10); // prescale = 64
	OCR1C=(uint8_t)(remainder);
	TIMSK |= (1 << TOIE1); // enable OVF interrupt
}

// verbindet Minuten und Stunden zu einer Zahl und schreibt in Screen rein
void show_time(uint8_t hour, uint8_t minute) 
{
	screen = (minute << 8) | hour;
}

// Bringt Led zum leuchten manuell
void set_manually_led(uint8_t pin)
{
	// 1 in S -> 4 -> Diode 1 wird angemacht (Erklärung oben)
	screen |= (1 << S[pin-1]);
}

// Macht Register und Leds an - Charlieplexing Logik
/*
schaltet eine spezifische LED ein, indem sie die Pins entsprechend der im M-Array definierten Konfiguration einstellt.
setzt einen Pin auf High, einen anderen auf Low, 
und lässt alle anderen Pins unverändert (d.h. sie bleiben als Input gesetzt und damit inaktiv).
*/
void led_on(uint8_t pin_with_offset)
{
	uint8_t x = (M[pin_with_offset] >> 3);
	uint8_t y = (M[pin_with_offset] & 7);
	DDRA |= ((1 << x) | (1 << y));
	PORTA = (PORTA | (1 << x)) & ~(1 << y);
}

// Buttons sind Eingänge
void button_init(void)
{
	// Eing�nge markieren und Pullup-Widerstand deaktivieren
	DDRA &= ~((1 << BUTTON1) | (1 << BUTTON2) | (1 << BUTTON3));
	PORTA &= ~((1 << BUTTON1) | (1 << BUTTON2) | (1 << BUTTON3));
}

void button_handler(void) // 10 Hz
{
	for(uint8_t i = 0; i < 3; i++)
	{
		uint8_t temp;
		// Button3 = 0 = i
		// Erhöhe integrator
		if (CHECK_BIT(BUTTON1_PIN, i)) {
			/* PIN is high */
			// (0.075 * 40) = 3 = Maximum
			// 0000 0000
			// Wir haben drei Buttons, und alle müssen jeweils parallel bis 3 zählen können
			// Button A, B, C
			// 00CC BBAA -> 0000 00AA = 0 (AA == 00) , 1 (AA == 01), 2 (AA == 11)

			// Button nicht gedrückt: 00
			// Beginn button wird gedrückt: 01 (nach 0,1 Sekunden)
			// button wird weiter gedrückt: 10 (nach 0,2 Sekunden)
			// button wird weiter gedrückt: 11 (nach 0,3 Sekunden)

			if (GET_BITS(buttons_integrator, i * 2, 2) < (uint8_t) MAXIMUM) {
				temp = GET_BITS(buttons_integrator, i * 2, 2) + 1;
				buttons_integrator &= ~((uint8_t) MAXIMUM << i * 2);
				buttons_integrator |= (temp << i * 2);
			}

		}
		// Senke integrator
		else if (GET_BITS(buttons_integrator, i * 2, 2) > 0) {
			temp = GET_BITS(buttons_integrator, i * 2, 2) - 1;
			buttons_integrator &= ~((uint8_t) MAXIMUM << i * 2);
			buttons_integrator |= (temp << i * 2);
		}
		// Button nicht gedrückt
		if (GET_BITS(buttons_integrator, i * 2, 2) == 0) {
			buttons_modus &= ~(1 << i);
		}
		// = 3 -> Button wird gedrückt
		else if (GET_BITS(buttons_integrator, i * 2, 2) >= (uint8_t) MAXIMUM)
		{
			buttons_modus |= (1 << i);
			buttons_integrator |= ((uint8_t) MAXIMUM << i * 2); /* defensive code if integrator got corrupted */
		}
	}
}

// Interrupt wird getriggert wenn 124, wenn das 125 mal passiert, dann ist eine Sekunde rum
ISR (TIMER0_COMPA_vect) {
	time.beforesecond++;
	if (time.beforesecond == 124){
		time.second++;
		time.beforesecond = 0;
		if (time.second == 60) {
			time.second = 0;
			time.minute++;
			if (time.minute == 60) {
				time.minute = 0;
				time.hour++;
				if (time.hour == 24){
					time.hour = 0;
				}
			}
		}
	}
}

// Interrupt Timer fürs Charlieplexing und handlen der Buttons
// Overflow Interrupt - feuert immer wenn Maximal Wert erreicht
ISR(TIMER1_OVF_vect) { // wird 495x pro Sekunde ausgef�hrt: alle 0,00202 Sekunden wird das gestartet. Duration muss << 0,00202 Sekunden
	interrupt_leds_off();
	
	// Button wird nicht so oft gebraucht wie Leds -> abzählen
	if (++button_timer_counter == (uint8_t)button_timer) {
		button_handler();  // has to be called at least once every 100ms
		button_timer_counter = 0;
	}
	// Routine f�r Schirmiteration
	// Soll die Led gerade leuchten, wenn nicht überspringe
	if (CHECK_BIT(screen, S[active_diode])) {
		led_on(active_diode);
	}
	if (active_diode >= 10) {
		active_diode = 0;
	} else {
		active_diode++;
	}
}

int main(void) {
	cli(); // deaktiviert alle Interrupts
	led_init();  // Initialisiert die LEDs
	button_init(); // Initialisiert die Kn�pfe
	timer_init(); // Initialisiert den Timer
	sei();  // Aktiviert globale Interrupts
		
	uint8_t display_mode = 0;
	volatile uint8_t processed_buttons = buttons_modus;

	time.minute = 59;
	time.hour = 23;

	while(1) { // läuft unendlich, gibt aber Interrupts die Zeit vorranschreiten regeln 
		show_time(time.hour, time.minute);  // Zeigt die aktuelle Zeit auf den LEDs an
		_delay_ms(200);
			// Displaymodus = 0 -> Uhrzeit, keine Einstellung
			// Displaymodus = 1 -> Einstellung Minuten
			// Displaymodus = 2 -> Einstellung Stunden
			
		if (CHECK_BIT(buttons_modus, BUTTON3) && !CHECK_BIT(processed_buttons, BUTTON3)) {
			display_mode++;
			if (display_mode == 3) {
				display_mode = 0;
			}
			processed_buttons |= (1 << BUTTON3);
		} else if (!CHECK_BIT(buttons_modus, BUTTON3)) {
			processed_buttons &= ~(1 << BUTTON3);
		}
			
		if (CHECK_BIT(buttons_modus, BUTTON1) && display_mode == 1) {
			time.minute++;
			if (time.minute == 60) {
				time.minute = 0;
			}
			processed_buttons |= (1 << BUTTON1);
		} else if (CHECK_BIT(buttons_modus, BUTTON1) && display_mode == 2) {
			time.hour++;
			if (time.hour == 24) {
				time.hour = 0;
			}
			processed_buttons |= (1 << BUTTON1);
		} else if (!CHECK_BIT(buttons_modus, BUTTON1)) {
			processed_buttons &= ~(1 << BUTTON1);
		}
			
		if (CHECK_BIT(buttons_modus, BUTTON2) && display_mode == 1) {
			time.minute--;
			if (time.minute > 59) {
				time.minute = 59;
			}
			processed_buttons |= (1 << BUTTON2);
		} else if (CHECK_BIT(buttons_modus, BUTTON2) && display_mode == 2) {
			time.hour--;
			if (time.hour > 23) {
				time.hour = 23;
			}
			processed_buttons |= (1 << BUTTON2); // processed_buttons = processed_buttons | (1 << BUTTON2);
		} else if (!CHECK_BIT(buttons_modus, BUTTON2)) {
			processed_buttons &= ~(1 << BUTTON2);
		}
	}

	return 0;
}


// Setzt Wert an Stelle 3 auf 1 ohne DDRA zu verändern
// DDRA |= (1 << 3)		00001000
// Setzt Wert an Stelle 3 auf 0 ohne DDRA zu verändern - egal was vorher war
// DDRA &= ~(1 << 3) 	00000000

// PORTA |= (1 << 3)
