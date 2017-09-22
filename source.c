#include<p18f2320.h>
#include<timers.h>
#include<pwm.h>

#pragma config OSC=INTIO1,FSCM=OFF,IESO=OFF,PWRT=OFF
#pragma config BOR=ON,BORV=42,WDT=OFF,WDTPS=1024
#pragma config MCLRE=ON,PBAD=DIG//TODO:,CCP2MX=C1*******************************************************************
#pragma config STVR=OFF,LVP=OFF,DEBUG=OFF
#pragma config CP0=OFF,CP1=OFF,CP2=OFF,CP3=OFF,CPB=OFF
#pragma config CPD=OFF,WRT0=OFF,WRT1=OFF,WRT2=OFF,WRT3=OFF
#pragma config WRTB=OFF,WRTC=OFF,WRTD=OFF,EBTR0=OFF
#pragma config EBTR1=OFF,EBTR2=OFF,EBTR3=OFF,EBTRB=OFF


//const char DIRECTION_LEFT=0b00100100;
//const char DIRECTION_RIGHT=0b00001010;
const char DIRECTION_FORWARD=0b011000;

const char SENSOR_LEFT=0b1000;
const char SENSOR_RIGHT=1;
const char SENSOR_MIDDLE=0b10;
const char SENSOR_RIGHT_FORWARD=0b100;

const char SENSOR_MASK=0b1111;
const char LR_SENSOR_MASK=0b1000|1;

const double PI=3.14159265359;
const unsigned int MEASURE_SPEED_TIMER0_INTERVAL=3035;//速度測定タイマー0.5秒間隔
const unsigned int MAX_SPEED_COUNT=200;//TODO:値決定
const unsigned int PWM_MAX_WIDTH=1023;

volatile unsigned int left_rotaryencoder_deltacounter;
volatile unsigned int right_rotaryencoder_deltacounter;

unsigned int actual_left_speed;
unsigned int actual_right_speed;
unsigned int goal_left_speed;
unsigned int goal_right_speed;
unsigned int left_PWM_width;
unsigned int right_PWM_width;

void set_speed(double left_speed_rate,double right_speed_rate){
        goal_left_speed=left_speed_rate*MAX_SPEED_COUNT;
        goal_right_speed=right_speed_rate*MAX_SPEED_COUNT;
        left_PWM_width=left_speed_rate*PWM_MAX_WIDTH;
        right_PWM_width=right_speed_rate*PWM_MAX_WIDTH;
        SetDCPWM1(left_PWM_width);
        SetDCPWM2(right_PWM_width);
}

enum direction{
        left,
        right
};

unsigned int mm_to_count(unsigned int mm){
        return mm*36/65;//センサ円周65mm カウント36
}

void debug_output(){
	LATB=LATB|((PORTA<<4) & 0b11110000);
}

//FIXME:pwm=0 0に何掛けても0
//FIXME:pwm=0,1023 CCPR1LがCCPR1Hにコピーされない
//TODO:他にvolatileが必要な個所は

volatile unsigned int left_rotaryencoder_distance;
volatile unsigned int right_rotaryencoder_distance;
void walk(unsigned int distance){
        distance=mm_to_count(distance);//HACK:ループ内に入れると再計算してパフォーマンス低下の可能性があるため
        set_speed(0.5,0.5);
        //直進なので左右の動きは同じと仮定して右だけを見る
        right_rotaryencoder_distance=0;
        while(right_rotaryencoder_distance<distance){
		debug_output();
	}
}

void turn(enum direction direction){
        const unsigned int ARC_LENGTH_COUNT=mm_to_count(PI*92/4);
        switch(direction){
                case left:
                        set_speed(0,0.5);
                        right_rotaryencoder_distance=0;
                        while(right_rotaryencoder_distance<ARC_LENGTH_COUNT){
				debug_output();
			}
                        break;
                case right:
                        set_speed(0.5,0);
                        left_rotaryencoder_distance=0;
                        while(left_rotaryencoder_distance<ARC_LENGTH_COUNT){
				debug_output();
			}
                        break;
        }
        set_speed(0.5,0.5);
}

void crank_turn();//[2058] call of function without prototype抑止
void crank_turn(){
	LATCbits.LATC6=1;//デバッグ出力
        walk(184);
        turn(left);
	LATCbits.LATC6=0;//デバッグ出力
}

void change_lane();//[2058] call of function without prototype抑止
void change_lane(){
	LATCbits.LATC7=1;//デバッグ出力
        walk(204);
        turn(right);
        while((PORTA & SENSOR_LEFT)==SENSOR_LEFT){//左側に黒い線が来るまで
		debug_output();
	}
        turn(left);
	LATCbits.LATC7=0;//デバッグ出力
}

void main(){
        OSCCON=0x62;

        TRISA=0b1111;//フォトリフレクタ
        TRISB=0b11;//デバッグ用LEDとロータリーエンコーダー
        TRISC=0;//デバッグ用ピンとモーター出力
        ADCON1=0b1111;

        INTCON2bits.TMR0IP=0;
        OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8);
        WriteTimer0(MEASURE_SPEED_TIMER0_INTERVAL);

        LATC=DIRECTION_FORWARD;
        set_speed(0.5,0.5);

        OpenTimer2(TIMER_INT_OFF & T2_PS_1_4 & T2_POST_1_1);
        OpenPWM1(0xFE);
        OpenPWM2(0xFE);

        RCONbits.IPEN=1;//割込み優先順位有効
        INTCONbits.GIEH=1;//高位割込み有効
        INTCONbits.GIEL=1;//低位割込み有効
        INTCONbits.INT0IE=1;
        INTCON3bits.INT1IE=1;
        INTCONbits.TMR0IE=1;

        while(1){
		debug_output();

                if((PORTA & SENSOR_MASK)==(SENSOR_MIDDLE|SENSOR_LEFT)){
                        change_lane();
                }else if((PORTA & LR_SENSOR_MASK)==0){
                        crank_turn();
                }else if((PORTA & LR_SENSOR_MASK)==SENSOR_LEFT){
                        set_speed(0.5,0.1);
                }else if((PORTA & LR_SENSOR_MASK)==SENSOR_RIGHT){
                        set_speed(0.1,0.5);
                }else if((PORTA & LR_SENSOR_MASK)==(SENSOR_LEFT|SENSOR_RIGHT)){
                        set_speed(0.5,0.5);
		}
        }
}

#pragma code //おそらく不要
#pragma interrupt rotaryencoder_handler
void rotaryencoder_handler(){
        if(INTCONbits.INT0IF==1){
                left_rotaryencoder_deltacounter++;
                left_rotaryencoder_distance++;

                LATBbits.LATB3^=1;//ロータリーエンコーダー デバッグ用出力

                INTCONbits.INT0IF=0;
        }
        if(INTCON3bits.INT1IF==1){
                right_rotaryencoder_deltacounter++;
                right_rotaryencoder_distance++;

                LATBbits.LATB3^=1;//ロータリーエンコーダー デバッグ用出力

                INTCON3bits.INT1IF=0;
        }			
}
#pragma code rotaryencoder_vector_addr=0x8
void rotaryencoder_vector(){
        _asm goto rotaryencoder_handler _endasm
}

#pragma code
#pragma interruptlow timer0_handler
void timer0_handler(){
        INTCONbits.TMR0IF=0;
        WriteTimer0(MEASURE_SPEED_TIMER0_INTERVAL);

        actual_left_speed=left_rotaryencoder_deltacounter;
        actual_right_speed=right_rotaryencoder_deltacounter;

        left_PWM_width=left_PWM_width*goal_left_speed/actual_left_speed;//HACK:乗算代入演算子キャストされてしまう
        right_PWM_width=right_PWM_width*goal_right_speed/actual_right_speed;//HACK:乗算代入演算子キャストされてしまう

        if(left_PWM_width>PWM_MAX_WIDTH){
                left_PWM_width=PWM_MAX_WIDTH;
        }
        if(right_PWM_width>PWM_MAX_WIDTH){
                right_PWM_width=PWM_MAX_WIDTH;
        }

        //SetDCPWM1(left_PWM_width);
        //SetDCPWM2(right_PWM_width);

        left_rotaryencoder_deltacounter=0;
        right_rotaryencoder_deltacounter=0;
}
#pragma code timer0_vector_addr=0x18
void timer0_vector(){
        _asm goto timer0_handler _endasm
}
