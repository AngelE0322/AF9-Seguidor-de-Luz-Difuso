/*
 AF9 PERCEPCIÓN
 Blas Eduardo Escobar Rodríguez 1903990
 José Ángel Salinas Rodriguez
*/
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#if !defined(CONFIG_IDF_TARGET_ESP32P4)
#error "Selecciona ESP32P4 Dev Module. Este montaje es para ESP32-P4."
#endif

constexpr uint8_t PIN_LDR_I = 20, PIN_LDR_D = 21, PIN_SERVO = 2;
constexpr uint32_t PERIODO_MS = 100, REPORTE_MS = 500;
constexpr uint8_t PWM_BITS = 14;
constexpr float POS_MIN = 45, POS_MAX = 135;
constexpr float GANANCIA = 0.35f;       // Reduce la correccion por ciclo.
constexpr float PASO_MAX = 1.0f;        // Maximo nominal por 100 ms.
constexpr float BANDA_LUZ = 3.0f;       // Diferencia normalizada para detener.
constexpr float LUZ_MINIMA = 5.0f;      // No buscar en oscuridad.
constexpr float ALPHA = 0.25f;          // Filtro exponencial de ADC.
constexpr float RANGO_CAL_MIN = 150.0f; // Cuentas ADC, resolucion 12 bits.
// Pulsos conservadores. No implican que cada servo tenga 180 grados exactos.
constexpr float PULSO_0_US = 1000, PULSO_180_US = 2000;

// BEGIN_FUZZY
// Mismos conjuntos, reglas y malla de salida (401 puntos) que Colab.
float triangular(float x, float a, float b, float c) {
  if (x == b) return 1.0f;
  if (x <= a || x >= c) return 0.0f;
  return (x < b) ? (x-a)/(b-a) : (c-x)/(c-b);
}

float difuso(float izquierda, float derecha) {
  const float entradas[3][3] = {{0,0,50},{0,50,100},{50,100,100}};
  const float salidas[5][3] = {{-5,-5,-2.5f},{-5,-2.5f,0},
                             {-1,0,1},{0,2.5f,5},{2.5f,5,5}};
  const uint8_t reglas[3][3] = {{2,3,4},{1,2,3},{0,1,2}};
  float mi[3], md[3], fuerza[5] = {0,0,0,0,0};
  for (int k=0;k<3;k++) {
    mi[k]=triangular(izquierda,entradas[k][0],entradas[k][1],entradas[k][2]);
    md[k]=triangular(derecha,entradas[k][0],entradas[k][1],entradas[k][2]);
  }
  for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
    int k=reglas[i][j];
    fuerza[k]=fmaxf(fuerza[k],fminf(mi[i],md[j]));
  }
  // Centroide por integracion exacta de cada segmento lineal de la malla,
  // equivalente al centroide de skfuzzy sobre esa misma malla muestreada.
  double area=0, momento=0, xAnterior=0, yAnterior=0;
  for (int n=0;n<=400;n++) {
    float x=-5.0f+n/40.0f, y=0;
    for (int k=0;k<5;k++) {
      float mu=triangular(x,salidas[k][0],salidas[k][1],salidas[k][2]);
      y=fmaxf(y,fminf(fuerza[k],mu));
    }
    if (n>0) {
      double h=double(x)-xAnterior;
      double a=h*(yAnterior+y)*0.5;
      area+=a;
      momento+=xAnterior*a+h*h*(yAnterior*0.5+(double(y)-yAnterior)/3.0);
    }
    xAnterior=x; yAnterior=y;
  }
  if (area<=0) return NAN;
  float resultado=float(momento/area);
  return fabsf(resultado)<1e-6f ? 0.0f : resultado;
}
// END_FUZZY

Preferences memoria;
bool memoriaOK=false, calibrado=false, pendienteOSC=false;
bool automatico=false, servoActivo=false, filtroListo=false, csv=true;
float oscuroI=0, oscuroD=0, claroI=0, claroD=0;
float adcI=0, adcD=0, filtroI=0, filtroD=0;
float luzI=-1, luzD=-1, salidaDifusa=0, aplicado=0, posicion=90;
int signoServo=1;
uint32_t ultimoControl=0, ultimoReporte=0;
const char* estado="MONITOR";
char linea[80]; size_t usados=0; bool desbordada=false;

float limitar(float x,float lo,float hi) { return fminf(hi,fmaxf(lo,x)); }
float leerPromedio(uint8_t pin,int n=16) {
  analogRead(pin);  // Descarta primera muestra tras seleccionar el canal.
  uint32_t suma=0;
  for(int k=0;k<n;k++) { suma+=analogRead(pin); delayMicroseconds(100); }
  return float(suma)/n;
}
bool calibracionValida() {
  return isfinite(oscuroI) && isfinite(oscuroD) && isfinite(claroI) && isfinite(claroD)
    && oscuroI>=0 && oscuroD>=0 && claroI<4090 && claroD<4090
    && claroI-oscuroI>=RANGO_CAL_MIN && claroD-oscuroD>=RANGO_CAL_MIN;
}
float normalizar(float valor,float oscuro,float claro) {
  return limitar(100.0f*(valor-oscuro)/(claro-oscuro),0,100);
}
bool escribirServo(float nueva) {
  nueva=limitar(nueva,POS_MIN,POS_MAX);
  if(!servoActivo) {
    if(!ledcAttach(PIN_SERVO,50,PWM_BITS)) {
      automatico=false; estado="ERROR_PWM";
      Serial.println("# ERROR: no se pudo iniciar LEDC a 50 Hz / 14 bits.");
      return false;
    }
    servoActivo=true;
  }
  float pulso=PULSO_0_US+(PULSO_180_US-PULSO_0_US)*nueva/180.0f;
  uint32_t duty=uint32_t(lroundf(pulso*float((1UL<<PWM_BITS)-1)/20000.0f));
  if(!ledcWrite(PIN_SERVO,duty)) {
    automatico=false; estado="ERROR_PWM";
    Serial.println("# ERROR: no se pudo escribir PWM."); return false;
  }
  posicion=nueva; return true;
}
void cabecera() {
  Serial.println("ms,adcI,adcD,luzI,luzD,difuso_deg,aplicado_deg,pos_ordenada,estado");
}
void ayuda() {
  Serial.println("# AF9 ESP32-P4: 115200 baudios, Nueva linea o Ambos NL/CR.");
  Serial.println("# CENTRO: activa servo en 90; I/D: prueba +/-5 grados nominales.");
  Serial.println("# INVERTIR: cambia y guarda el signo fisico del servo.");
  Serial.println("# OSC: guardar lecturas bajas de ambas LDR (tapadas).");
  Serial.println("# LUZ: guardar lecturas altas con igual luz y guardar calibracion.");
  Serial.println("# AUTO: seguir luz. STOP/MONITOR: detener seguimiento y mantener consigna.");
  Serial.println("# OFF: detener y retirar PWM (servo deja de sostener).");
  Serial.println("# TEST 80 20: comparar nucleo con Colab, sin mover servo; pausa AUTO.");
  Serial.println("# ESTADO: ver calibracion. CSV ON / CSV OFF: activar/desactivar registro.");
  Serial.println("# AYUDA o ?: mostrar comandos. Tras reiniciar siempre empieza en MONITOR.");
}
void mostrarEstado() {
  Serial.printf("# Calibrado=%s; signo=%d; servo=%s; auto=%s\n",calibrado?"SI":"NO",
      signoServo,servoActivo?"ON":"OFF",automatico?"SI":"NO");
  Serial.printf("# OSC I=%.1f D=%.1f; LUZ I=%.1f D=%.1f\n",oscuroI,oscuroD,claroI,claroD);
  Serial.printf("# Ganancia=%.2f; periodo=%lu ms; banda=%.1f; limites=%.0f..%.0f nominales\n",
      GANANCIA,(unsigned long)PERIODO_MS,BANDA_LUZ,POS_MIN,POS_MAX);
}
void procesar(char* cmd) {
  while(*cmd==' ' || *cmd=='\t') cmd++;
  size_t n=strlen(cmd);
  while(n && isspace((unsigned char)cmd[n-1])) cmd[--n]=0;
  for(size_t k=0;k<n;k++) cmd[k]=toupper((unsigned char)cmd[k]);
  if(!n) return;
  if(!strcmp(cmd,"?") || !strcmp(cmd,"AYUDA")) ayuda();
  else if(!strcmp(cmd,"ESTADO")) mostrarEstado();
  else if(!strcmp(cmd,"CSV ON")) { csv=true; cabecera(); }
  else if(!strcmp(cmd,"CSV OFF")) csv=false;
  else if(!strcmp(cmd,"STOP") || !strcmp(cmd,"MONITOR")) {
    automatico=false; aplicado=0; estado="MONITOR";
    Serial.println("# Seguimiento detenido; se mantiene la ultima consigna si PWM esta activo.");
  } else if(!strcmp(cmd,"OFF")) {
    automatico=false; aplicado=0;
    if(servoActivo) ledcDetach(PIN_SERVO);
    servoActivo=false; pinMode(PIN_SERVO,OUTPUT); digitalWrite(PIN_SERVO,LOW);
    estado="MONITOR"; Serial.println("# PWM retirado.");
  } else if(!strcmp(cmd,"CENTRO")) {
    automatico=false; aplicado=0; estado="MANUAL";
    if(escribirServo(90)) Serial.println("# Centro ordenado: 90 nominales / 1500 us.");
  } else if(!strcmp(cmd,"I") || !strcmp(cmd,"D")) {
    automatico=false; aplicado=0; estado="MANUAL";
    if(!servoActivo) { Serial.println("# Primero envia CENTRO."); return; }
    float paso=(cmd[0]=='I'?-5.0f:5.0f)*signoServo;
    if(escribirServo(posicion+paso)) Serial.printf("# Consigna=%.2f; comprobar sentido fisico.\n",posicion);
  } else if(!strcmp(cmd,"INVERTIR")) {
    automatico=false; aplicado=0; signoServo=-signoServo; estado="MONITOR";
    if(memoriaOK && memoria.putInt("signo",signoServo)!=sizeof(int))
      Serial.println("# Aviso: signo cambiado solo en RAM; no se pudo guardar.");
    Serial.printf("# Signo=%d. Repite CENTRO e I.\n",signoServo);
  } else if(!strcmp(cmd,"OSC")) {
    automatico=false; aplicado=0; estado="CALIBRANDO";
    oscuroI=leerPromedio(PIN_LDR_I,128); oscuroD=leerPromedio(PIN_LDR_D,128);
    pendienteOSC=true; calibrado=false; filtroListo=false;
    Serial.printf("# OSC: I=%.1f D=%.1f. Ahora ilumina ambas por igual y envia LUZ.\n",oscuroI,oscuroD);
  } else if(!strcmp(cmd,"LUZ")) {
    automatico=false; aplicado=0;
    if(!pendienteOSC) { Serial.println("# Primero envia OSC con ambas LDR tapadas."); return; }
    claroI=leerPromedio(PIN_LDR_I,128); claroD=leerPromedio(PIN_LDR_D,128);
    calibrado=calibracionValida(); filtroListo=false;
    Serial.printf("# LUZ: I=%.1f D=%.1f.\n",claroI,claroD);
    if(!calibrado) {
      estado="CAL_INVALIDA";
      Serial.println("# Calibracion invalida: LUZ debe superar OSC al menos 150 cuentas.");
      Serial.println("# Evita ADC >=4090; aleja la linterna o reduce las resistencias de 10k.");
      Serial.println("# Revisa conexiones y repite OSC/LUZ. AUTO permanece bloqueado.");
    } else {
      pendienteOSC=false; estado="MONITOR";
      bool guardada=false;
      if(memoriaOK) {
        memoria.putBool("valida",false);
        guardada=memoria.putFloat("oi",oscuroI)==4 && memoria.putFloat("od",oscuroD)==4
          && memoria.putFloat("ci",claroI)==4 && memoria.putFloat("cd",claroD)==4;
        if(guardada) guardada=memoria.putBool("valida",true)==1;
      }
      Serial.println(guardada?"# Calibracion guardada en memoria.":"# Calibracion valida SOLO en RAM; repetir al reiniciar.");
    }
  } else if(!strcmp(cmd,"AUTO")) {
    if(!calibrado) { Serial.println("# Falta calibracion: OSC y LUZ."); return; }
    if(!servoActivo) { Serial.println("# Primero CENTRO y comprueba I/D."); return; }
    automatico=true; estado="AUTO"; ultimoControl=millis();
    Serial.println("# Seguimiento activo. STOP para detener.");
  } else if(!strncmp(cmd,"TEST ",5)) {
    automatico=false; aplicado=0; estado="MONITOR";
    float i,d; char extra;
    if(sscanf(cmd+5,"%f %f %c",&i,&d,&extra)==2 && isfinite(i) && isfinite(d)
       && i>=0 && i<=100 && d>=0 && d<=100)
      Serial.printf("# TEST %.3f %.3f => %.6f grados difusos (sin mover).\n",i,d,difuso(i,d));
    else Serial.println("# Uso: TEST 80 20 (dos numeros de 0 a 100).");
  } else Serial.println("# Comando no reconocido. Envia AYUDA.");
}
void leerComandos() {
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='\n' || c=='\r') {
      if(desbordada) Serial.println("# Linea demasiado larga; descartada.");
      else if(usados) { linea[usados]=0; procesar(linea); }
      usados=0; desbordada=false;
    } else if(!desbordada) {
      if(usados<sizeof(linea)-1) linea[usados++]=c;
      else { desbordada=true; usados=0; }
    }
  }
}
void setup() {
  Serial.begin(115200); delay(800);
  pinMode(PIN_SERVO,OUTPUT); digitalWrite(PIN_SERVO,LOW);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LDR_I,ADC_11db);
  analogSetPinAttenuation(PIN_LDR_D,ADC_11db);
  memoriaOK=memoria.begin("af9ldr",false);
  if(memoriaOK) {
    signoServo=memoria.getInt("signo",1);
    if(signoServo!=1 && signoServo!=-1) signoServo=1;
    oscuroI=memoria.getFloat("oi",0); oscuroD=memoria.getFloat("od",0);
    claroI=memoria.getFloat("ci",0); claroD=memoria.getFloat("cd",0);
    calibrado=memoria.getBool("valida",false) && calibracionValida();
  }
  ayuda(); mostrarEstado(); cabecera();
}
void loop() {
  leerComandos();
  uint32_t ahora=millis();
  if(uint32_t(ahora-ultimoControl)<PERIODO_MS) return;
  ultimoControl=ahora;
  adcI=leerPromedio(PIN_LDR_I); adcD=leerPromedio(PIN_LDR_D);
  if(!filtroListo) { filtroI=adcI; filtroD=adcD; filtroListo=true; }
  else { filtroI+=ALPHA*(adcI-filtroI); filtroD+=ALPHA*(adcD-filtroD); }
  aplicado=0; salidaDifusa=0; luzI=-1; luzD=-1;
  if(calibrado) {
    luzI=normalizar(filtroI,oscuroI,claroI); luzD=normalizar(filtroD,oscuroD,claroD);
    salidaDifusa=difuso(luzI,luzD);
    if(automatico) {
      if(!isfinite(salidaDifusa)) { automatico=false; estado="ERROR_DIFUSO"; }
      else if(adcI>=4090 || adcD>=4090) estado="SATURADO";
      else if(fmaxf(luzI,luzD)<LUZ_MINIMA) estado="OSCURO";
      else if(fabsf(luzI-luzD)<=BANDA_LUZ) estado="EQUILIBRIO";
      else {
        float paso=limitar(GANANCIA*salidaDifusa,-PASO_MAX,PASO_MAX)*signoServo;
        float nueva=limitar(posicion+paso,POS_MIN,POS_MAX);
        float cambio=nueva-posicion;
        estado=fabsf(cambio)<1e-6f?"LIMITE":"AUTO";
        if(escribirServo(nueva)) aplicado=cambio;
      }
    }
  }
  if(csv && uint32_t(ahora-ultimoReporte)>=REPORTE_MS) {
    ultimoReporte=ahora;
    Serial.printf("%lu,%.1f,%.1f,%.2f,%.2f,%.5f,%.5f,%.2f,%s\n",(unsigned long)ahora,
      adcI,adcD,luzI,luzD,salidaDifusa,aplicado,posicion,estado);
  }
}
