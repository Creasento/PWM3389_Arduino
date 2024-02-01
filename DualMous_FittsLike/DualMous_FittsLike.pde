import processing.serial.*;
import java.util.*;
import java.lang.reflect.Method;

import java.time.*;
import java.time.format.DateTimeFormatter;

import java.awt.*;
import java.awt.event.*;

Serial sp;

String mouse_info;
int cpi;
int sensor_pos;
int def_cpi = 0;
int def_pos = 100;
float cpi_multiplier;
int lf = 10;

Point cursor_pos = new Point(0, 0);
Point target = new Point(0, 0);
Point prev = new Point(0, 0);

boolean test = false;
boolean clicked = false;
boolean success_prev = false;

int setDelay = 0;
int frameRate = 240;

int nRepeat = 1;
int cycle = 11;

int[] distances = {200, 400, 600};
int[] widths = {30, 60, 90};

int current_cond = 0;
Experiment current_exp;

int cnt = 0;
int cnt_trial = 0;
int cnt_success = 0;

PrintWriter Main_Logger;
PrintWriter Pos_Logger;
PrintWriter Sen_Logger;
String log_id = "";

ArrayList<Experiment> cond = new ArrayList<Experiment>();
ArrayList<Point> dots = new ArrayList<Point>();

LocalTime prev_time;
LocalTime curr_time;
long taken;


void setup() {

  LocalDateTime now = LocalDateTime.now();
  DateTimeFormatter fmt = DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm_ss");
  log_id = now.format(fmt);

  for (int i = 0; i < nRepeat; i++) {
    for (int D : distances) {
      for (int W : widths) {
        cond.add(new Experiment(D, W));
      }
    }
  }

  long seed = System.nanoTime();
  Collections.shuffle(cond, new Random(seed));

  current_cond = 0;
  current_exp = cond.get(current_cond);

  printArray(Serial.list());
  //String portName = Serial.list()[Serial.list().length - 1];

  sp = new Serial(this, "COM6", 9600);
  sp.clear();

  if (def_cpi != 0) setCPI(sp, def_cpi);
  setPOS(sp, def_pos);

  getMouseInfo(sp);

  Pos_Logger = StartLogging_Pos();
  Pos_Logger.println("Distance,Width,Count,Success,CursorX,CursorY,Time(nanosec)");

  cpi_multiplier = (float)cpi / 12000;

  cursor_pos = new Point(0, 0);

  fullScreen();
  frameRate(frameRate);
  noCursor();

  textSize(24);
  ellipseMode(CENTER);
  rectMode(CENTER);

  println("READY");
}


void draw()
{
  // screen fadeout when setDelay > 0
  if (setDelay > 0)
  {
    background(30);
    if (cnt_trial == 0) {
      textAlign(CENTER, CENTER);
      text("Ready", width/2, height/2);
      text(setDelay/240, width/2, height/2 + 40);
    }
    setDelay--;
    return;
  }

  background(255);
  noStroke();

  fill(25);
  textAlign(LEFT, TOP);
  text("Current Mode | " + (test ? "Test" : "Practice"), 10, 10);
  text("Session | " + (current_cond + 1) + " / "  + cond.size(), 10, 75);
  text(mouse_info.replace(":", "|"), 10, 109);

  if (!test) {
    textAlign(RIGHT, TOP);
    float acc = ((float)cnt_success / cnt_trial) * 100;
    text("Accuracy | " + acc + "%", width - 10, 10);
    text("( " + cursor_pos.x + " , " + cursor_pos.y + " )", width - 10, 44);
  }

  // set coordinate system w.r.t. screen center
  int center_x = width/2;
  int center_y = height/2;
  pushMatrix();
  translate(center_x, center_y); // set origin at the center of the screen

  // Fitts' Law targets drawing
  float W = current_exp.W;
  float D = current_exp.D;

  prev = new Point(0, 0);

  fill(127);

  // formula: make the target to be opposite of the previous target, like 0-3-1-4-2-0 when cycle=7
  int prev_target_num = ((cnt-1)%2 * cycle/2 + (cnt-1)/2 + (cnt-1)%2) % cycle;
  int target_num = (cnt%2 * cycle/2 + cnt/2 + cnt%2) % cycle;

  for (int i=0; i<cycle; i++)    // draw all the targets, split by 360deg / cycle
  {
    float angle = radians(i * 360.0 / cycle);
    float x0 = 0;
    float y0 = -1*D/2;
    float rot_x = x0 * cos(angle) - y0 * sin(angle);
    float rot_y = x0 * sin(angle) + y0 * cos(angle);

    if (target_num == i) {
      target = new Point(round(rot_x), round(rot_y));
    }
    if (prev_target_num == i)
    {
      prev = new Point(round(rot_x), round(rot_y));
    }
    ellipse(rot_x, rot_y, W, W);
  }

  // draw target in green
  fill(0, 255, 0);
  ellipse(target.x, target.y, W, W);

  // draw previous target in red when failed in previous click
  if (!success_prev && (prev.x != 0 || prev.y != 0) && !test)
  {
    fill(255, 0, 0);
    ellipse(prev.x, prev.y, W, W);
  }

  // Mouse position update
  while (sp.available() > 0)
  {
    String response = sp.readStringUntil(lf);
    if (response == null)
      continue;

    // parse mouse messages
    String[] message = splitTokens(trim(response), "\t");
    if (message.length != 8)
      continue;
    long timestamp = Long.parseLong(message[0]);
    int f_dx = int(message[1]);  // front sensor dx, dy
    int f_dy = int(message[2]);
    int r_dx = int(message[3]);  // rear sensor dx/dy
    int r_dy = int(message[4]);
    int m_dx = int(message[5]);  // generate mouse movement dx/dy
    int m_dy = int(message[6]);
    int btn_state = int(message[7]);

    // when logger is opened, trial has been started, and not ended
    cursor_pos.translate(m_dx, m_dy); // update the cursor with the mouse log event

    if (Main_Logger != null && cnt > 0 && cnt <= cycle+1)
    {
      Main_Logger.println(cnt+"\t"+(target.x-prev.x)+"\t"+(target.y-prev.y)+"\t"
        +trim(response)+"\t"
        +cursor_pos.x+"\t"+cursor_pos.y+"\t"
        +target.x+"\t"+target.y+"\t"
        +prev.x+"\t"+prev.y);
      Sen_Logger.println(timestamp+"\t"+  f_dx+"\t"+f_dy+"\t"+r_dx+"\t"+r_dy+"\t"+btn_state);
    }
    //sensor logger
    if (sp.available() > 0)
    {
      Optional<String> senLog = Optional.ofNullable(sp.readStringUntil('\n'));
      if (senLog.isPresent())
      {
        String senStr1 = senLog.get();
        String senStr2 = senLog.orElse("null");
        print(senStr1+senStr2);
        delay(1); // buffer
      }
      else
      {
        //Pass
      }

    }

    // 0b#$  => #: right button, $: left button (1 when clicked)
    if ((btn_state&0x01) > 0)
    {
      Clicked();
    } else
    {
      Released();
    }
  }
  // constrain the cursor to be in the screen boundary
  cursor_pos.move(constrain(cursor_pos.x, -width/2, width/2), constrain(cursor_pos.y, -height/2, height/2));

  fill(0);

  rect(cursor_pos.x, cursor_pos.y, 20, 2);
  rect(cursor_pos.x, cursor_pos.y, 2, 20);

  popMatrix();
}


void Clicked() {

  if (clicked != true) OnClick();
  clicked = true;
}


void Released() {

  if (clicked != false) OnRelease();
  clicked = false;
}


void OnClick() {

  if (Main_Logger == null && setDelay == 0) {
    Main_Logger = StartLogging_Main(cond, current_cond);
  }
  if (Sen_Logger == null && setDelay == 0) {
    Sen_Logger = StartLogging_Sen(cond, current_cond);
  }

  if (cnt == 0) {
    prev_time = LocalTime.now();
  } else {
    curr_time = LocalTime.now();
    Duration duration = Duration.between(prev_time, curr_time);
    taken = duration.getSeconds() * 1000000000 + duration.getNano();
    prev_time = LocalTime.now();
  }

  cnt++;
  cnt_trial++;
  success_prev = dist(cursor_pos.x, cursor_pos.y, target.x, target.y) <= current_exp.W/2;


  if (success_prev) cnt_success++;
  dots.add(new Point(cursor_pos));

  if (cnt > 1) {
    String Log = current_exp.toString().replace("_", ",") + "," + (cnt-1) + "," + (success_prev ? "T" : "F") + "," + cursor_pos.x + "," + cursor_pos.y + "," + taken ;

    Pos_Logger.println(Log);
    Pos_Logger.flush();
  }

  if (cnt > cycle) {
    cnt = 0;
    StopLogging(Main_Logger);
    StopLogging(Sen_Logger);
    Main_Logger = null;
    Sen_Logger = null;
    dots.clear();
    current_cond++;

    if (current_cond > cond.size()-1) {
      StopLogging(Pos_Logger);
      exit();
      println("EXIT");
      float acc = (float)cnt_success / cnt_trial;
      println("Accuracy : " + acc);
    } else {
      current_exp = cond.get(current_cond);
    }

    setDelay = int(frameRate * 0.25);
  } else {
    //cursor_pos.move(target.x, target.y);
  }
}


void OnRelease() {

  return;
}


int setCPI(Serial port, int newCPI) {

  port.write("s\n");
  port.clear();

  port.write("C" + newCPI + "\n");
  port.clear();

  String read = "";

  while (splitTokens(trim(read)).length != 2) {
    read = null;
    while (read == null) read = port.readStringUntil(lf);
  }

  cpi_multiplier = (float)newCPI / 12000;

  port.write("S\n");
  return int(splitTokens(trim(read))[0]);
}


int setPOS(Serial port, int newPOS) {

  port.write("s\n");
  port.clear();

  port.write("P" + newPOS + "\n");
  port.clear();

  String read = "";

  while (splitTokens(trim(read)).length != 1) {
    read = null;
    while (read == null) read = port.readStringUntil(lf);
  }

  port.write("S\n");
  return int(splitTokens(trim(read))[0]);
}


void getMouseInfo(Serial port) {

  port.write("s\n");
  port.clear();

  port.write("R\n");
  port.clear();

  String read = "";

  while (splitTokens(trim(read)).length != 2) {
    read = null;
    while (read == null) read = port.readStringUntil(lf);
  }

  sensor_pos = int(splitTokens(trim(read))[1]);
  mouse_info = "Current Sensor Position : " + sensor_pos + "\n";

  read = null;
  while (read == null) read = port.readStringUntil(lf);

  cpi = int(splitTokens(trim(read))[1]);
  mouse_info += "Current CPI : " + cpi;

  port.clear();
  port.write("S/n");
}


PrintWriter StartLogging_Pos() {

  String CurrentMode = test ? "Main" : "Practice";
  String msinfo = cpi + "_" + sensor_pos;
  //log file name information (mouse pos)
  String LogName = "./Logs/" + log_id + "_" + CurrentMode + "_" + msinfo + "_logs/Pos_values.csv";

  PrintWriter pw = createWriter(LogName);
  return pw;
}


PrintWriter StartLogging_Main(ArrayList<Experiment> conditions, int cond_num) {

  Experiment exp = conditions.get(cond_num);
  String CurrentMode = test ? "Main" : "Practice";
  String msinfo = cpi + "_" + sensor_pos;
  //log file name information (mouse log)
  String logName = "./Logs/" + log_id + "_" + CurrentMode + "_" + msinfo + "_" + "logs/" + exp + "_" + (cond_num + 1) + ".log";
  PrintWriter pw = createWriter(logName);
  cnt = 0;

  return pw;
}

PrintWriter StartLogging_Sen(ArrayList<Experiment> conditions, int cond_num) {

  Experiment exp = conditions.get(cond_num);
  String CurrentMode = test ? "Main" : "Practice";
  String msinfo = cpi + "_" + sensor_pos;
  //log file name information (mouse log)
  String logName = "./Logs/" + log_id + "_" + CurrentMode + "_" + msinfo + "_" + "logs/" + "Sensor" + ".log";
  PrintWriter sw = createWriter(logName);
  cnt = 0;

  return sw;
}

void StopLogging(PrintWriter pw) {

  if (pw != null) {
    pw.flush();
    pw.close();
  }

  pw = null;
}


void keyPressed() {

  if (keyCode == UP) {
    setCPI(sp, cpi + 100);
    getMouseInfo(sp);
  } else if (keyCode == DOWN) {
    setCPI(sp, cpi - 100);
    getMouseInfo(sp);
  } else if (keyCode == RIGHT) {
    setPOS(sp, sensor_pos + 5);
    getMouseInfo(sp);
  } else if (keyCode == LEFT) {
    setPOS(sp, sensor_pos - 5);
    getMouseInfo(sp);
  } else if (key == 'M' || key == 'm') {
    test = !test;
  } else if (key == ESC) {
    key = 0;
    StopLogging(Main_Logger);
    StopLogging(Pos_Logger);
    StopLogging(Sen_Logger);
    exit();
  }
}
