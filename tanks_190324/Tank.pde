/** Ida Söderberg, Magnus Palmstierna och Paulina Lagebjer Kekkonen (Grupp 5) **/ //<>// //<>// //<>//

import java.util.Comparator; //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>//
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.List;
import java.util.LinkedList;
import java.util.Collections;
import java.util.Arrays;

class Tank extends Sprite {
  int id;
  //String name; //Sprite
  int team_id;

  PVector acceleration;
  PVector velocity;
  //PVector position; //Sprite

  float rotation;
  float rotation_speed;

  Team team;
  PImage img;
  //float diameter; //Sprite
  //float radius; //Sprite

  float maxrotationspeed;
  float maxspeed;
  float maxforce;

  int health;// 3 är bra, 2 är mindre bra, 1 är immobilized, 0 är destroyed.
  boolean isImmobilized; // Tanken kan snurra på kanonen och skjuta, men inte förflytta sig.
  boolean isDestroyed; // Tanken är död.

  //PVector hitArea;

  PVector startpos;
  PVector positionPrev; //spara temp senaste pos.

  Node startNode; // noden där tanken befinner sig.
  Node currentNode;
  Node target;
  boolean hasTarget; // Används just nu för att kunna köra "manuellt" (ai har normalt target).
  PVector targetPosition; // Används vid förflyttning mot target.
  float targetHeading; // Används vid rotation mot en target.
  PVector sensor_targetPosition;

  PVector[] otherTanks  = new PVector[5];
  PVector distance3_sensor;

  ArrayList listOfActions; // Används ännu inte.

  float heading; // Variable for heading!

  // variabler som anger vad tanken håller på med.
  boolean backward_state;
  boolean forward_state;
  boolean turning_right_state;
  boolean turning_left_state;
  boolean turning_turret_right_state;
  boolean turning_turret_left_state;
  boolean stop_state;
  boolean stop_turning_state;
  boolean stop_turret_turning_state;
  boolean patrolling;
  boolean tankAhead;

  boolean idle_state; // Kan användas när tanken inte har nåt att göra.
  boolean collidedWithTree;
  boolean collidedWithTank;

  boolean isMoving; // Tanken är i rörelse.
  boolean isRotating; // Tanken håller på att rotera.
  boolean isColliding; // Tanken håller på att krocka.
  boolean isAtHomebase;
  boolean userControlled; // Om användaren har tagit över kontrollen.
  boolean isReporting;
  boolean isReportingInHomebase;
  boolean okayToGoNextStepHome;

  boolean hasShot; // Tanken kan bara skjuta om den har laddat kanonen, hasShot=true.
  CannonBall ball;

  float s = 2.0;
  float image_scale;

  boolean isSpinning; // Efter träff snurrar tanken runt, ready=false.
  boolean isReady; // Tanken är redo för action efter att tidigare blivit träffad.
  int remaining_turns;
  float heading_saved; // För att kunna återfå sin tidigare heading efter att ha snurrat.

  Turret turret;

  // Tank sensors
  private HashMap<String, Sensor> mappedSensors = new HashMap<String, Sensor>();
  private ArrayList<Sensor> sensors = new ArrayList<Sensor>();

  protected ArrayList<Sensor> mySensors = new ArrayList<Sensor>();

  // List of traversed areas
  private HashMap<Node, Integer> patrolled = new HashMap<Node, Integer>();
  private ArrayList<Node> homeNodes = new ArrayList<Node>();
  private ArrayList<Node> enemyNodes = new ArrayList<Node>();
  private Node lastVisitedNode;
  private Node lastSeenEnemy;
  PVector vectorTarget; // Used for patrolling
  Direction lastDir; // Used for reporting

  // Used to make sure tank stays still for three seconds after reporting to homebase
  StopWatchTimer timer;

  //Shortest path home 
  private LinkedList<Node> pathHome = new LinkedList<Node>();
  //**************************************************
  Tank(int id, Team team, PVector _startpos, float diameter, CannonBall ball) {
    println("*** NEW TANK(): [" + team.getId()+":"+id+"]");
    this.id = id;
    this.team = team;
    this.team_id = this.team.getId();

    this.name = "tank";

    this.startpos = new PVector(_startpos.x, _startpos.y);
    this.position = new PVector(this.startpos.x, this.startpos.y);
    this.velocity = new PVector(0, 0);

    this.acceleration = new PVector(0, 0);
    this.positionPrev = new PVector(this.position.x, this.position.y); //spara temp senaste pos.
    this.targetPosition = new PVector(this.position.x, this.position.y); // Tanks har alltid ett target.

    this.startNode = grid.getNearestNode(this.startpos);
    this.currentNode = startNode;


    if (this.team.getId() == 0) this.heading = radians(0); // "0" radians.
    if (this.team.getId() == 1) this.heading = radians(180); // "3.14" radians.

    this.targetHeading = this.heading; // Tanks har alltid en heading mot ett target.
    this.hasTarget = false;

    this.diameter = diameter;
    this.radius = this.diameter/2; // For hit detection.

    this.backward_state = false;
    this.forward_state = false;
    this.turning_right_state = false;
    this.turning_left_state = false;
    this.turning_turret_right_state = false;
    this.turning_turret_left_state = false;
    this.stop_state = true;
    this.stop_turning_state = true;
    this.stop_turret_turning_state = true;
    // Under test
    this.isMoving = false;
    this.isRotating = false;
    this.isAtHomebase = true;
    this.idle_state = true;
    this.isReporting = false;
    this.isReportingInHomebase = false;
    this.okayToGoNextStepHome = true;

    this.ball = ball;
    this.hasShot = false;
    this.maxspeed = 3; //3;
    this.maxforce = 0.1;
    this.maxrotationspeed = radians(3);
    this.rotation_speed = 0;
    this.image_scale = 0.5;
    this.isColliding = false;
    this.patrolling = false;
    this.tankAhead = false;

    //this.img = loadImage("tankBody2.png");
    this.turret = new Turret(this.diameter/2);

    this.radius = diameter/2;

    this.health = 3;// 3 är bra, 2 är mindre bra, 1 är immobilized, 0 är oskadliggjord.
    this.isReady = true; // Tanken är redo för action.
    this.isImmobilized = false; // Tanken kan snurra på kanonen och skjuta, men inte förflytta sig.
    this.isDestroyed = false; // Tanken är död.

    this.isSpinning = false;
    this.remaining_turns = 0;
    this.heading_saved = this.heading;

    this.ball.setColor(this.team.getColor());
    this.ball.setOwner(this);

    Node[][] nodes = grid.getAllNodes();
    for (int i = 0; i < nodes.length; i++) {
      for (int j = 0; j < nodes[i].length; j++) {
        if (nodes[i][j].x > team.homebase_x && nodes[i][j].x < team.homebase_x+team.homebase_width && nodes[i][j].y > team.homebase_y && nodes[i][j].y < team.homebase_x+team.homebase_height) {
          patrolled.put(nodes[i][j], -1);
          homeNodes.add(nodes[i][j]);
        } else if (nodes[i][j].x > width-150 && nodes[i][j].x < width && nodes[i][j].y > height-350 && nodes[i][j].y < height) {
          enemyNodes.add(nodes[i][j]);
        }
      }
    }

    initializeSensors();
  }


  //**************************************************
  int getId() {
    return this.id;
  }

  //**************************************************
  //String getName(){
  //  return this.name;
  //}

  //**************************************************
  float getRadius() {
    return this.radius;
  }

  //**************************************************
  float getHeadingInDegrees() {
    return degrees(this.heading);
  }

  //**************************************************
  float getHeading() {
    return this.heading;
  }

  //**************************************************
  // Anropas då användaren tar över kontrollen av en tank.
  void takeControl() {
    println("*** Tank[" + team.getId()+"].takeControl()");
    stopMoving_state();
    this.userControlled = true;
  }

  //**************************************************
  // Anropas då användaren släpper kontrollen av en tank.
  void releaseControl() {
    println("*** Tank[" + team.getId()+"].releaseControl()");
    stopMoving_state();
    idle_state = true;

    this.userControlled = false;
  }

  //**************************************************
  // Används ännu inte.
  PVector getRealPosition() {
    return this.position;
  }

  //************************************************** 
  // Returns the Sensor with the specified ID

  public Sensor getSensor(String ID) {
    return mappedSensors.get(ID);
  }

  //************************************************** 
  // Add your Sensor.

  public void addSensor(Sensor s) {
    mySensors.add(s);
  }

  //**************************************************
  //Register a sensor inside this robot, with the given ID

  protected void registerSensor(Sensor sensor, String ID) {
    mappedSensors.put(ID, sensor);
    sensors.add(sensor);
  }

  //**************************************************
  protected void initializeSensors() {

    SensorDistance ultrasonic_front = new SensorDistance(this, 0f);
    registerSensor(ultrasonic_front, "ULTRASONIC_FRONT");

    //SensorDistance ultrasonic_back = new SensorDistance(this, 180f);
    //registerSensor(ultrasonic_back, "ULTRASONIC_BACK");

    /*
     SensorCompass compass = new SensorCompass(game, this);
     registerSensor(compass, "COMPASS");
     
     SensorDistance ultrasonic_left = new SensorDistance(game, this, 270f);
     registerSensor(ultrasonic_left, "ULTRASONIC_LEFT");
     
     SensorDistance ultrasonic_right = new SensorDistance(game, this, 90f);
     registerSensor(ultrasonic_right, "ULTRASONIC_RIGHT");
     
     SensorDistance ultrasonic_front = new SensorDistance(game, this, 0f);
     registerSensor(ultrasonic_front, "ULTRASONIC_FRONT");
     
     SensorDistance ultrasonic_back = new SensorDistance(game, this, 180f);
     registerSensor(ultrasonic_back, "ULTRASONIC_BACK");
     */
  }

  //**************************************************

  SensorReading readSensor_distance(Sensor sens) {
    //println("*** Tank.readSensorDistance()");

    Sprite df = sens.readValue().obj();

    return sens.readValue();
  }

  //**************************************************
  void readSensors() {
    /*
     println("*** Tank[" + team.getId()+"].readSensors()");
     println("sensors: " + sensors);
     
     for (Sensor s : mySensors) {
     if (s.tank == this) {
     PVector sens = (s.readValue().obj().position);
     
     //println("============");
     //println("("+sens.x + " , "+sens.y+")");
     //ellipse(sens.x, sens.y, 10,10);
     if (sens != null) {
     line(this.position.x,this.position.y, sens.x, sens.y);
     println("Tank" + this.team.getId() + ":"+this.id + " ( " + sens.x + ", "+ sens.y + " )");
     
     }
     }
     */
  }

  //**************************************************
  void spin(int antal_varv) {
    println("*** Tank[" + team.getId()+"].spin(int)");
    if (!this.isSpinning) {
      this.heading_saved = this.heading;
      isSpinning = true; 
      this.remaining_turns = antal_varv;
    }
  }

  //**************************************************
  // After calling this method, the tank can shoot.
  void loadShot() {
    println("*** Tank[" + team.getId()+":"+id+"].loadShot() and ready to shoot.");

    this.hasShot = true;
    this.ball.loaded();
  }

  //**************************************************
  void testCollisionSensor() {
  }

  //**************************************************
  void fire() {
    // Ska bara kunna skjuta när den inte rör sig.
    if (this.stop_state) {
      println("*** Tank[" + this.team.getId()+":"+this.id+"].fire()");

      if (this.hasShot) {
        println("! Tank["+ this.getId() + "] – PANG.");
        this.hasShot = false;

        PVector force = PVector.fromAngle(this.heading + this.turret.heading);
        force.mult(10);
        this.ball.applyForce(force);

        shoot(this.id); // global funktion i huvudfilen

        soundManager.playSound("tank_firing");
        //soundManager.playSound("blast");
      } else {
        println("! Tank["+ this.getId() + "] – You have NO shot loaded and ready.");
      }
    } else {
      println("! Tank["+ this.getId() + "] – The tank must stand STILL to shoot.");
    }
  }

  //**************************************************
  // Anropad från den cannonBall som träffat.
  final boolean takeDamage() {
    println("*** Tank["+ this.getId() + "].takeDamage()");

    if (!this.isDestroyed) {
      this.health -= 1;

      println("! Tank[" + team.getId()+":"+id+"] has been hit, health is now "+ this.health);

      stopMoving_state();
      resetTargetStates();

      if (!this.isImmobilized) {
        if (this.health == 1) {
          this.isImmobilized = true;
        }
      }

      if (this.health <= 0) {
        this.health = 0;
        this.isDestroyed = true;
        this.isSpinning  = false;
        this.isReady = false;

        return true;
      }

      spin(3);
      this.isReady = false; // Efter träff kan inte tanken utföra action, så länge den "snurrar".


      return true;
    }

    return false; // ingen successfulHit omtanken redan är destroyed.
  }

  //**************************************************
  // Anropad från sin egen cannonBall efter träff.
  final void successfulHit() {
    this.team.messageSuccessfulHit();
  }

  //**************************************************
  // Det är denna metod som får tankens kanon att svänga vänster.
  void turnTurretLeft_state() {
    if (this.stop_state) { 
      if (!this.turning_turret_left_state) {
        println("*** Tank[" + getId() + "].turnTurretLeft_state()");
        this.turning_turret_right_state = false;
        this.turning_turret_left_state = true;
        this.stop_turret_turning_state = false;
      }
    } else {
      println("Tanken måste stå still för att kunna rotera kanonen.");
    }
  }

  //**************************************************
  void turnTurretLeft() {
    this.turret.turnLeft();
  }

  //**************************************************
  // Det är denna metod som får tankens kanon att svänga höger.
  void turnTurretRight_state() {
    if (this.stop_state) { 
      if (!this.turning_turret_right_state) {
        println("*** Tank[" + getId() + "].turnTurretRight_state()");
        this.turning_turret_left_state = false;
        this.turning_turret_right_state = true;
        this.stop_turret_turning_state = false;
      }
    } else {
      println("Tanken måste stå still för att kunna rotera kanonen.");
    }
  }

  //**************************************************
  void turnTurretRight() {
    this.turret.turnRight();
  }

  //**************************************************
  // Det är denna metod som får tankens kanon att sluta rotera.
  void stopTurretTurning_state() {
    if (!this.stop_turret_turning_state) {
      println("*** Tank[" + getId() + "].stopTurretTurning_state()");
      this.turning_turret_left_state = false;
      this.turning_turret_right_state = false;
      this.stop_turret_turning_state = true;
    }
  }

  //**************************************************
  // Det är denna metod som får tanken att svänga vänster.
  void turnLeft_state() {
    this.stop_turning_state = false;
    this.turning_right_state = false;

    if (!this.turning_left_state) {
      println("*** Tank[" + getId() + "].turnLeft_state()");
      this.turning_left_state = true;
    }
  }

  //**************************************************
  void turnLeft() {
    //println("*** Tank[" + getId()+"].turnLeft()");

    if (this.hasTarget && abs(this.targetHeading - this.heading) < this.maxrotationspeed) {
      this.rotation_speed -= this.maxforce;
    } else {
      this.rotation_speed += this.maxforce;
    }
    if (this.rotation_speed > this.maxrotationspeed) {
      this.rotation_speed = this.maxrotationspeed;
    }
    this.heading -= this.rotation_speed;
  }

  //**************************************************
  // Det är denna metod som får tanken att svänga höger.
  void turnRight_state() {
    this.stop_turning_state = false;
    this.turning_left_state = false;

    if (!this.turning_right_state) {
      println("*** Tank[" + getId() + "].turnRight_state()");
      this.turning_right_state = true;
    }
  }

  //**************************************************  
  void turnRight() {
    //println("*** Tank[" + getId() + "].turnRight()");

    if (this.hasTarget && abs(this.targetHeading - this.heading) < this.maxrotationspeed) {
      this.rotation_speed -= this.maxforce;
    } else {
      this.rotation_speed += this.maxforce;
    }
    if (this.rotation_speed > this.maxrotationspeed) {
      this.rotation_speed = this.maxrotationspeed;
    }
    this.heading += this.rotation_speed;
  }

  //**************************************************  
  void turnRight(PVector targetPos) {
    println("*** Tank[" + getId() + "].turnRight(PVector)");
    PVector desired = PVector.sub(targetPos, position);  // A vector pointing from the position to the target

    desired.setMag(0);
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    applyForce(steer);
  }

  //**************************************************  
  void stopTurning() {
    println("*** Tank[" + getId()+"].stopTurning()");
    this.rotation_speed = 0;
    arrivedRotation();
  }

  //**************************************************
  // Det är denna metod som får tanken att sluta svänga.
  void stopTurning_state() {
    if (!this.stop_turning_state) {
      println("*** Tank[" + getId() + "].stopTurning_state()");
      this.turning_left_state = false;
      this.turning_right_state = false;
      this.stop_turning_state = true;

      println("! Tank[" + getId() + "].stopTurning_state() – stop_turning_state=true");
    }
  }

  //**************************************************
  void moveTo(float x, float y) {
    println("*** Tank["+ this.getId() + "].moveTo(" +x +", "+ y+")");

    moveTo(new PVector(x, y));
  }

  //**************************************************
  void moveTo(PVector coord) {
    //println("*** Tank["+ this.getId() + "].moveTo(PVector)");
    if (!isImmobilized) {
      println("*** Tank["+ this.getId() + "].moveTo(" +coord.x +", "+ coord.y+")");


      this.idle_state = false;
      this.isMoving = true;
      this.stop_state = false;

      this.targetPosition.set(coord);

      this.hasTarget = true;
    }
  }

  //**************************************************
  void moveBy(float x, float y) {
    println("*** Tank["+ this.getId() + "].moveBy(float x, float y)");

    moveBy(new PVector(x, y));
  }

  //**************************************************
  void moveBy(PVector coord) {
    println("*** Tank["+ this.getId() + "].moveBy(PVector)");

    PVector newCoord = PVector.add(this.position, coord);
    PVector nodevec = grid.getNearestNodePosition(newCoord);

    moveTo(nodevec);
  }

  //**************************************************
  // Det är denna metod som får tanken att gå framåt.
  void moveForward_state() {
    println("*** Tank[" + getId() + "].moveForward_state()");

    if (!this.forward_state) {
      this.acceleration.set(0, 0, 0);
      this.velocity.set(0, 0, 0);

      this.forward_state = true;
      this.backward_state = false;
      this.stop_state = false;
    }
  }

  //**************************************************
  void moveForward() {
    //println("*** Tank[" + getId() + "].moveForward()");

    // Offset the angle since we drew the ship vertically
    float angle = this.heading; // - PI/2;
    // Polar to cartesian for force vector!
    PVector force = new PVector(cos(angle), sin(angle));
    force.mult(0.1);
    applyForce(force);
  }

  //**************************************************
  void moveForward(int numSteps) {
  }

  //**************************************************
  // Det är denna metod som får tanken att gå bakåt.
  void moveBackward_state() {
    println("*** Tank[" + getId() + "].moveBackward_state()");
    this.stop_state = false;
    this.forward_state = false;

    if (!this.backward_state) {
      println("! Tank[" + getId() + "].moveBackward_state() – (!this.backward_state)");
      this.acceleration.set(0, 0, 0);
      this.velocity.set(0, 0, 0);
      this.backward_state = true;
    }
  }

  //**************************************************
  void moveBackward() {
    println("*** Tank[" + getId() + "].moveBackward()");
    // Offset the angle since we drew the ship vertically
    float angle = this.heading - PI; // - PI/2;
    // Polar to cartesian for force vector!
    PVector force = new PVector(cos(angle), sin(angle));
    force.mult(0.1);
    applyForce(force);
  }

  //**************************************************
  void stopMoving() {
    println("*** Tank[" + getId() + "].stopMoving()");

    this.acceleration.set(0, 0, 0);
    this.velocity.set(0, 0, 0);

    this.isMoving = false;  

    resetTargetStates();
  }

  //**************************************************
  // Det är denna metod som får tanken att sluta åka framåt eller bakåt.
  // "this.stop_state" anropas 
  void stopMoving_state() {
    //println("stopMoving_state() ");

    if (!this.stop_state) {
      //println("*** Tank[" + getId() + "].stopMoving_state()");

      resetMovingStates();
      stopMoving();
    }
  }

  //**************************************************
  void resetAllMovingStates() {
    println("*** Tank[" + getId() + "].resetAllMovingStates()");
    this.stop_state = true;
    this.backward_state = false;
    this.forward_state = false;

    this.backward_state = false;
    this.forward_state = false;
    this.turning_right_state = false;
    this.turning_left_state = false;
    this.turning_turret_right_state = false;
    this.turning_turret_left_state = false;
    this.stop_state = true;
    this.stop_turning_state = true;
    this.stop_turret_turning_state = true;

    this.velocity = new PVector(0, 0);
    this.acceleration = new PVector(0, 0);
  }

  //**************************************************
  void resetMovingStates() {
    println("*** Tank[" + getId() + "].resetMovingStates()");
    this.stop_state = true;
    this.backward_state = false;
    this.forward_state = false;
  }

  //**************************************************
  void resetTargetStates() {
    println("*** Tank[" + getId() + "].resetTargetStates()");
    this.targetPosition = new PVector(this.position.x, this.position.y);

    this.targetHeading = this.heading; // Tanks har alltid en heading mot ett target.
    this.hasTarget = false;
  }

  //**************************************************
  void updatePosition() {

    this.positionPrev.set(this.position); // spara senaste pos.

    this.velocity.add(this.acceleration);
    this.velocity.limit(this.maxspeed);
    this.position.add(this.velocity);
    this.acceleration.mult(0);
  }

  //**************************************************
  // Newton's law: F = M * A
  void applyForce(PVector force) {
    this.acceleration.add(force);
  }

  //**************************************************
  public void destroy() {
    println("*** Tank.destroy()");
    //dead = true;
    this.isDestroyed = true;
  }

  //**************************************************

  void rotating() {
    //println("*** Tank["+ this.getId() + "].rotating()");
    if (!isImmobilized) {

      if (this.hasTarget) {
        float diff = this.targetHeading - this.heading;

        if ((abs(diff) <= radians(0.5))) {
          this.isRotating = false;
          this.heading = this.targetHeading;
          this.targetHeading = 0.0;
          this.hasTarget = false;
          stopTurning_state();
          arrivedRotation();
        } else if ((diff) > radians(0.5)) {

          turnRight_state();
        } else if ((diff) < radians(0.5)) {
          turnLeft_state();
        }
      }
    }
  }

  //**************************************************

  void rotateTo(float angle) {
    println("*** Tank["+ this.getId() + "].rotateTo(float): "+angle);

    if (!isImmobilized) {

      this.heading = angle;

      // Hitta koordinaten(PVector) i tankens riktning
      Sensor sens = getSensor("ULTRASONIC_FRONT");
      PVector sens_pos = (sens.readValue().obj().position);
      PVector grid_pos = grid.getNearestNodePosition(sens_pos);
      rotateTo(grid_pos); // call "rotateTo(PVector)"
    }
  }

  //**************************************************

  void rotateTo(PVector coord) {
    println("*** Tank["+ this.getId() + "].rotateTo(PVector) – ["+(int)coord.x+","+(int)coord.y+"]");

    if (!isImmobilized) {

      this.idle_state = false;
      this.isMoving = false;
      this.isRotating = true;
      this.stop_state = false;
      this.hasTarget = true;


      PVector target = new PVector(coord.x, coord.y);
      PVector me = new PVector(this.position.x, this.position.y);

      // Bestäm headin till target.
      PVector t = PVector.sub(target, me);
      this.targetHeading = t.heading();
    }
  }

  //**************************************************
  // A method that calculates a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  void arrive() {

    // rotera tills heading mot target.
    PVector desired = PVector.sub(this.targetPosition, this.position);  // A vector pointing from the position to the target
    float d = desired.mag();
    // If arrived

    // Scale with arbitrary damping within 100 pixels
    if (d < 100) {
      float m = map(d, 0, 100, 0, maxspeed);
      desired.setMag(m);
    } else {
      desired.setMag(maxspeed);
    }

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    applyForce(steer);

    if (d < 1) {
      arrived();
    }
  }

  //**************************************************
  // Tanken meddelas om att tanken är redo efter att blivit träffad.
  void readyAfterHit() {
    println("*** Tank["+ this.getId() + "].readyAfterHit()");

    if (!this.isDestroyed) {
      this.isReady = true; // Efter träff kan inte tanken utföra action, så länge den "snurrar".
    }
  }

  //**************************************************
  // Tanken meddelas om kollision med trädet.
  void arrivedRotation() {
    println("*** Tank["+ this.getId() + "].arrivedRotation()");
    stopTurning_state();
    this.isMoving = false;
  }

  //**************************************************
  void arrived() {
    println("*** Tank["+ this.getId() + "].arrived()");
    this.isMoving = false;  
    okayToGoNextStepHome = true;
    stopMoving_state();
  }

  //**************************************************
  // Är tänkt att överskuggas (override) i subklassen.
  void updateLogic() {
  }

  //**************************************************
  // Called from game
  final void update() {

    // Om tanken fortfarande lever.
    if (!this.isDestroyed) {
      // Om tanken har blivit träffad och håller på och snurrar runt.
      int spinning_speed = 5;
      if (this.isSpinning) {
        if (this.remaining_turns > 0) {
          this.heading += rotation_speed * spinning_speed; 

          if (this.heading > (this.heading_saved + (2 * PI))||(this.heading == this.heading_saved)) {

            this.remaining_turns -= 1;
            this.heading = this.heading_saved;
          }
        } else {

          this.heading = this.heading_saved;
          this.remaining_turns = 0;
          this.isSpinning = false;
          this.isReady = true;

          this.idle_state = true;
        }
      } else {

        // Om tanken är redo för handling och kan agera.
        if (!this.isImmobilized && this.isReady) {  

          if (isReporting && okayToGoNextStepHome) {
            takeOneMoreStepHome();
          } else if (isReportingInHomebase) {
            // If we are done with reporting, go back to patrolling.

            if (timer != null && timer.seconds() >= 3) {
              println("TIME: "+ timer.seconds());

              timer.stop();
              timer = null;
              lastVisitedNode=null;
              isReportingInHomebase = false;
              isReporting = false;
              tankAhead = false;
              startPatrolling();
              println("EFTER: ");
            }
          }

          if (patrolling) {
            keepPatrolling();
          }

          // Om tanken är i rörelse.
          if (this.isMoving) {

            this.heading = this.velocity.heading();
            arrive();
          }


          // Om tanken ska stanna, men ännu inte gjort det.
          if (this.stop_state && !this.idle_state) {

            resetTargetStates(); // Tank
            resetAllMovingStates(); // Tank 
            this.idle_state = true;

            println("! Tank[" + getId() + "].update() – idle_state = true");
          }

          // Om tanken håller på och rotera.
          if (this.isRotating) {
            rotating();
          }

          // ----------------
          // state-kontroller
          if (this.forward_state) {
            moveForward();
          }
          if (this.backward_state) {
            moveBackward();
          }
          if (this.turning_right_state) {
            turnRight();
          }
          if (this.turning_left_state) {
            turnLeft();
          }

          if (this.stop_state && !this.isMoving && this.hasTarget) {
            println("Tank["+ this.getId() + "], vill stanna!");
            //this.stop_state = false;
            stopMoving();
          }
          if (this.stop_turning_state && !this.isMoving && this.hasTarget) {
            println("Tank["+ this.getId() + "], vill sluta rotera!");
            stopTurning();
          }
        } // end (!this.isImmobilized && this.isReady)



        // Om tanken är immobilized
        // Om tanken har laddat ett skott.
        if (this.hasShot) {
          this.ball.updateLoadedPosition(this.position);
        }

        //---------------
        // state-kontroller ...
        if (this.turning_turret_left_state) {
          turnTurretLeft();
        }
        if (this.turning_turret_right_state) {
          turnTurretRight();
        }


        readSensors();
      }
      updatePosition();
    }
  }

  //**************************************************
  // Anropas från spelet.
  void checkEnvironment() {
    //checkEnvironment_sensor();

    // Check for collisions with Canvas Boundaries
    float r = this.diameter/2;
    if ((this.position.y+r > height) || (this.position.y-r < 0) ||
      (this.position.x+r > width) || (this.position.x-r < 0)) {
      if (!this.stop_state) {
        this.position.set(this.positionPrev); // Flytta tillbaka.
        //println("***");
        stopMoving_state();
      }
    }

    if (
      position.x > team.homebase_x && 
      position.x < team.homebase_x+team.homebase_width &&
      position.y > team.homebase_y &&
      position.y < team.homebase_y+team.homebase_height) {
      if (!isAtHomebase) {
        isAtHomebase = true;

        // checks if reporting tank has just arrived to the homebase
        if (isReporting) {
          timer = new StopWatchTimer();
          timer.start();
          isReporting = false;
          isReportingInHomebase = true;
          patrolled.put(lastSeenEnemy, Integer.MAX_VALUE);
          for (Node n : getNeighboringNodes(lastSeenEnemy)) {
            patrolled.put(n, Integer.MAX_VALUE);
          }
          showGrid();
        }
        message_arrivedAtHomebase();
      }
    } else {
      isAtHomebase = false;
    }
  }

  // Tanken meddelas om att tanken är i hembasen.
  public void message_arrivedAtHomebase() {
    println("! Tank["+ this.getId() + "] – har kommit hem.");
  }

  // Tanken meddelas om kollision med trädet.
  public void message_collision(Tree other) {
    println("*** Tank["+ this.getId() + "].collision(Tree)");
    //println("Tank.COLLISION");
  }

  // Tanken meddelas om kollision med den andra tanken.
  public void message_collision(Tank other) {
    println("*** Tank["+ this.getId() + "].collision(Tank)");
    //println("Tank.COLLISION");
  }

  //**************************************************
  void checkCollision(Tree other) {
    //println("*** Tank.checkCollision(Tree)");
    // Check for collisions with "no Smart Objects", Obstacles (trees, etc.)

    // Get distances between the tree component
    PVector distanceVect = PVector.sub(other.position, this.position);

    // Calculate magnitude of the vector separating the tank and the tree
    float distanceVectMag = distanceVect.mag();

    // Minimum distance before they are touching
    float minDistance = this.radius + other.radius;

    if (distanceVectMag <= minDistance && !this.stop_state) {

      println("! Tank["+ this.getId() + "] – collided with Tree.");

      if (!this.stop_state) {
        this.position.set(this.positionPrev); // Flytta tillbaka.

        // Kontroll om att tanken inte "fastnat" i en annan tank. 
        distanceVect = PVector.sub(other.position, this.position);
        distanceVectMag = distanceVect.mag();
        collidedWithTree = true;
        if (distanceVectMag < minDistance) {
          println("! Tank["+ this.getId() + "] – FAST I ETT TRÄD");
        }

        stopMoving_state();
      }

      if (this.hasShot) {
        this.ball.updateLoadedPosition(this.positionPrev);
      }


      // Meddela tanken om att kollision med trädet gjorts.
      message_collision( other);//collision(Tree);
    }
  }

  //**************************************************
  // Called from environment
  // Keeps an array with vectors to the other tanks, so the tank object can access the other tanks when called for.
  void checkCollision(Tank other) {
    //println("*** Tank.checkCollision(Tank)");
    // Check for collisions with "Smart Objects", other Tanks.

    // Get distances between the tanks components
    PVector distanceVect = PVector.sub(other.position, this.position);

    // Calculate magnitude of the vector separating the tanks
    float distanceVectMag = distanceVect.mag();

    // Minimum distance before they are touching
    float minDistance = this.radius + other.radius;

    if (distanceVectMag <= minDistance) {
      println("! Tank["+ this.getId() + "] – collided with another Tank" + other.team_id + ":"+other.id);
      collidedWithTank = true;
      this.position.set(this.positionPrev); // Flytta tillbaka.
      if (!this.stop_state) {
        //this.position.set(this.positionPrev); // Flytta tillbaka.

        // Kontroll om att tanken inte "fastnat" i en annan tank. 
        distanceVect = PVector.sub(other.position, this.position);
        distanceVectMag = distanceVect.mag();


        if (distanceVectMag <= minDistance) {
          println("! Tank["+ this.getId() + "] – FAST I EN ANNAN TANK");
        }

        this.isMoving = false;  
        stopMoving_state();
      }

      if (this.hasShot) {
        this.ball.updateLoadedPosition(this.positionPrev);
      }


      // Meddela tanken om att kollision med den andra tanken gjorts.
      message_collision(other);
    }
    checkTankForward(other);
  }

  void setNode() {
    //setTargetPosition(this.position);
  }

  void displayInfo() {
    fill(230);
    rect(width - 151, 0, 150, 300);
    strokeWeight(1);
    fill(255, 0, 0);
    stroke(255, 0, 0);
    textSize(10);
    text("id: "+this.id+"\n"+
      "health: "+this.health+"\n"+
      "position: ("+(int)this.position.x +","+(int)this.position.y+")"+"\n"+
      "isMoving: "+this.isMoving+"\n"+
      "isSpinning : "+this.isSpinning +"\n"+
      "remaining_turns: "+this.remaining_turns +"\n"+
      "isReady : "+this.isReady +"\n"+
      "hasTarget : "+this.hasTarget +"\n"+
      "stop_state : "+this.stop_state +"\n"+
      "stop_turning_state : "+this.stop_turning_state +"\n"+
      "idle_state : "+this.idle_state +"\n"+
      "isDestroyed : "+this.isDestroyed +"\n"+
      "isImmobilized : "+this.isImmobilized +"\n"+
      "targetHeading : "+this.targetHeading +"\n"+
      "heading : "+this.heading +"\n"+
      "heading_saved: "+this.heading_saved +"\n"
      , width - 145, 35 );
  }

  //**************************************************
  void drawTank(float x, float y) {
    fill(this.team.getColor()); 

    if (this.team.getId() == 0) fill((((255/6) * this.health) *40 ), 50* this.health, 50* this.health, 255 - this.health*60);
    if (this.team.getId() == 1) fill(10*this.health, (255/6) * this.health, (((255/6) * this.health) * 3), 255 - this.health*60);

    if (this.userControlled) {
      strokeWeight(3);
    } else strokeWeight(1);

    ellipse(x, y, 50, 50);
    strokeWeight(1);
    line(x, y, x+25, y);

    fill(this.team.getColor(), 255); 
    this.turret.display();
  }

  //**************************************************
  final void display() {

    imageMode(CENTER);
    pushMatrix();
    translate(this.position.x, this.position.y);

    rotate(this.heading);

    //image(img, 20, 0);
    drawTank(0, 0);

    if (debugOn) {
      noFill();
      strokeWeight(2);
      stroke(255, 0, 0);
      ellipse(0, 0, this.radius * 2, this.radius * 2);

      //for (Sensor s : mySensors) {
      //  if (s.tank == this) {
      //     strokeWeight(2);
      //     stroke(0,0,255); 
      //     PVector sens = s.readValue1();
      //     println("============");
      //     println("("+sens.x + " , "+sens.y+")");
      //     //ellipse(sens.x, sens.y, 10,10);
      //  }
      //}
    }

    popMatrix();  

    if (pause) {
      PVector mvec = new PVector(mouseX, mouseY);
      PVector distanceVect = PVector.sub(mvec, this.position);
      float distanceVectMag = distanceVect.mag();
      if (distanceVectMag < getRadius()) {
        displayInfo();
      }
    }

    if (debugOn) {

      for (Sensor s : mySensors) {
        if (s.tank == this) {
          // Rita ut vad sensorn ser (target och linje dit.)
          strokeWeight(1);
          stroke(0, 0, 255); 
          PVector sens = (s.readValue().obj().position);

          //println("============");
          //println("("+sens.x + " , "+sens.y+")");
          //ellipse(sens.x, sens.y, 10,10);

          if ((sens != null && !this.isSpinning && !isImmobilized)) {
            line(this.position.x, this.position.y, sens.x, sens.y);
            ellipse(sens.x, sens.y, 10, 10);
            //println("Tank" + this.team.getId() + ":"+this.id + " ( " + sens.x + ", "+ sens.y + " )");
          }
        }
      }

      // Rita ut en linje mot target, och tank-id och tank-hälsa.
      strokeWeight(2);
      fill(255, 0, 0);
      stroke(255, 0, 0);
      textSize(14);
      text(this.id+":"+this.health, this.position.x + this.radius, this.position.y + this.radius);

      if (this.hasTarget) {
        strokeWeight(1);
        line(this.position.x, this.position.y, this.targetPosition.x, targetPosition.y);
      }
    }
  }

  //************************************************************************************

  /*void getView() {
   PVector viewForward = PVector.add(position, new PVector((float)Math.cos(heading), (float)Math.sin(heading)).mult(this.diameter*2));
   PVector viewLeft =PVector.add(position, new PVector((float)Math.cos(heading-90), (float)Math.sin(heading-90)).mult(this.diameter));
   PVector viewRight = PVector.add(position, new PVector((float)Math.cos(heading+90), (float)Math.sin(heading+90)).mult(this.diameter));
   ArrayList<Node> view = new ArrayList<Node>();
   view.add(grid.getNearestNode(position));
   view.add(grid.getNearestNode(viewForward));
   view.add(grid.getNearestNode(viewLeft));
   view.add(grid.getNearestNode(viewRight));
   for (Node n : view) {
   //println("get view" + n.x + n.y);
   assignCostValue(new ArrayList<Node>(), n);
   }
   }*/

  //Denna metod sätter tankens status till att den ska tillbaka till basen
  void report() {
    isReporting = true;
    tankAhead = false;

    findShortestPathHome();
  }

  //Denna metod påbörjar patrulleringen
  void startPatrolling() {
    new Thread() {
      public void run() {
        while (patrolling) {
          try {
            flock();
            Thread.sleep(1000);
          }
          catch(InterruptedException e) {
            continue;
          }
        }
      }
    }
    .start();
    patrolling = true;
    currentNode = grid.getNearestNode(position); //Hämtar noden som är närmast tankens nuvarande position
    if (lastVisitedNode == null) {
      lastVisitedNode = currentNode;
    }
    assignCostValue(new ArrayList<Node>(), currentNode);
    patrolling = true;
    Node target = getNextTarget();

    vectorTarget = new PVector(target.x, target.y);

    moveTo(vectorTarget);
  }
  void keepPatrolling() {

    //Om det är en tank framför en så ska tanken återvända till senast besökta noden som en reträtt
    if (tankAhead) {
      if (collidedWithTank) {
        vectorTarget = new PVector(lastVisitedNode.x, lastVisitedNode.y);
        moveTo(vectorTarget);
        collidedWithTank = false;
      }
      patrolling = false;
      isReporting = true;
      report();
      return;      

      //Om tanken kolliderat med ett träd så ska den gå tillbaka till senast besökta noden och ge noden som trädet sitter på ett värde som aldrig gör den mer värd än andra noder
    } else if (collidedWithTree) {
      currentNode = grid.getNearestNode(position);
      patrolled.put(target, Integer.MAX_VALUE);
      vectorTarget = new PVector(lastVisitedNode.x, lastVisitedNode.y);
      collidedWithTree = false;
      collidedWithTank = false;
      moveTo(vectorTarget);
      patrolled.put(currentNode, -1);
      assignCostValue(new ArrayList<Node>(), currentNode);
      startPatrolling();
    }

    //Om tanken är på en ny plats så ska den omdeklarera vad som är dens nuvarande nod och dess senaste nod
    //Sätt nuvarande noden i listan över patrullerade noder, med värdet -1 för att signalera att den inte är uträknad
    if (grid.getNearestNode(position) != currentNode) {
      lastVisitedNode = currentNode;
      currentNode = grid.getNearestNode(position);
      patrolled.put(currentNode, -1);
      assignCostValue(new ArrayList<Node>(), currentNode);
      target = getNextTarget();

      vectorTarget = new PVector(target.x, target.y);
      moveTo(vectorTarget);
    }
  }

  //Denna metod kollar en nods alla grannar, om nån av dem är opatrullerad får noden värdet 1, då den är 1 steg från ett mål

  void assignCostValue(ArrayList<Node> finished, Node n) {
    ArrayList<Node> neighbors = getNeighboringNodes(n);
    float r = this.diameter/2;
    for (Node temp : neighbors) {
      if (finished.contains(temp)) {
        continue;
      }
      if (!patrolled.containsKey(temp)) {
        if ((temp.position.y+r >= height) || (temp.position.y-r <= 0) ||
          (temp.position.x+r >= width) || (temp.position.x-r <= 0) ) { 
          patrolled.put(temp, Integer.MAX_VALUE);
        } else {
          patrolled.put(n, 1);
          backPropagate(n, 1);
        }
      } else {
        if (!finished.contains(n)) {
          finished.add(n);
        }

        assignCostValue(finished, temp);
      }
    }
  } 

  //Denna metod kollar alla grannar hos en nod och kollar om deras värde är dess värde är dess värde plus 1, eller lägre, och ändrar om inte.
  void backPropagate(Node n, int i) {
    for (Node temp : getNeighboringNodes(n)) {
      if (patrolled.containsKey(temp) && (patrolled.get(temp) == -1 || (patrolled.get(temp) > i+1 && patrolled.get(temp) != Integer.MAX_VALUE))) {
        patrolled.put(temp, i+1); 
        backPropagate(temp, i+1);
      }
    }
  }

  //Returnerar alla närliggande noder till noden current
  ArrayList<Node> getNeighboringNodes(Node current) {
    ArrayList<Node> neighbors = new ArrayList<Node>(); 
    for (int i = -1; i < 2; i++) {
      for (int j = -1; j < 2; j++) {
        if ((current.col + i >= 0) && (current.row + j >= 0) && !(i == 0 && j == 0) 
          && (current.col + i <= 14) && (current.row + j <= 14)) { 
          Node n = new Node(current.col + i, current.row + j, ((current.col + i)*grid.grid_size+grid.grid_size), ((current.row+j)*grid.grid_size+grid.grid_size)); 
          neighbors.add(n);
        }
      }
    }
    return neighbors;
  }

  //Returnerar nästa mål baserat på vilken av noderna som har lägst värde, om noden ej är patrullerad så returneras den direkt
  //Slumpfaktor är tillsatt för att testa att agenten agerar korrekt i olika rutter
  Node getNextTarget() {
    Node target = null; 
    ArrayList<Node> neighbors = getNeighboringNodes(currentNode); 
    Collections.shuffle(neighbors); 
    for (Node temp : neighbors) {
      if (!patrolled.containsKey(temp)) {
        return temp;
      }
      if ((target == null || patrolled.get(temp) < patrolled.get(target)) && patrolled.get(temp) > -1) {
        target = temp;
      }
    }
    checkEnvironment(); 
    return target;
  }
  void showGrid() {
    for (int i = 0; i < grid.nodes.length; i++) {
      for (int j = 0; j < grid.nodes[i].length; j++) {
        if (patrolled.containsKey(grid.nodes[j][i])) {
          if (patrolled.get(grid.nodes[j][i]) == Integer.MAX_VALUE) {
          } else {
          }
        } else {
        }
      }
    }
  }

  //Kollar om någon av fiendetankerna är framför tankens synfält
  void checkTankForward(Tank other) {
    if (!enemyNodes.contains(grid.getNearestNode(position))) {
      return;
    }
    PVector viewForward = PVector.add(position, new PVector((float)Math.cos(heading), (float)Math.sin(heading)).mult(this.diameter*2)); 
    PVector distanceVect = PVector.sub(other.position, viewForward); 
    float distanceVectMag = distanceVect.mag(); 
    float minDistance = this.radius + other.radius; 
    if (distanceVectMag <= minDistance) {
      tankAhead = true;
    }
  }

  //*****************************

  // Implementation of A* for finding the shortest path home
  // Inspired by pseudocode found at https://mat.uab.cat/~alseda/MasterOpt/AStar-Algorithm.pdf
  void findShortestPathHome() {
    Queue<AStarNode> openQueue = new PriorityQueue<AStarNode>(new HeuristicsComparator());
    LinkedList<AStarNode> closedList = new LinkedList<AStarNode>(); 
    openQueue.add(new AStarNode(currentNode, calculateHeuristics(currentNode), 0, null)); //adding start node
    AStarNode current;

    while (!openQueue.isEmpty()) {
      current = openQueue.poll();
      closedList.add(current);

      if (isNodeInHomeBase(current.node)) {

        //WE ARE DONE
        LinkedList<Node> finalPath = new LinkedList<Node>(); // the nodes which we actually move between
        LinkedList<Node> actualFinalPath = new LinkedList<Node>(); // the nodes that makes up the whole path home
        AStarNode currNode = closedList.getLast();

        //We use the direction to check if the tank can go straigth to a further node without stopping on nodes between
        Direction dir = null;
        Direction prevDir = null;

        // The lastDir variable is used to make sure that we get into the homebase
        lastDir = getDirection(currNode.node, currNode.visitedThrough.node);

        while (currNode != null) {
          prevDir = dir;
          if (currNode.visitedThrough != null) {
            dir = getDirection(currNode.node, currNode.visitedThrough.node);
          }
          if (dir == null ||  dir != prevDir) { 
            finalPath.addFirst(currNode.node);
          }
          actualFinalPath.addFirst(currNode.node);               
          currNode = currNode.visitedThrough;
        }
        pathHome = finalPath;

        return;
      }

      // få närliggande som vi känner till från current 
      // ska ligga i patrolled
      List<Node> children = grid.getNodesNeighbours(current.node);

      for (Node n : children) {
        if (patrolled.containsKey(n) && !(n.content instanceof Tree)) {
          double gValue = calculateGValue(current, n);
          double hValue = calculateHeuristics(n);
          double fValue = gValue+hValue;

          //Does openQueue contain the current node? If it does and the heuristic for that in the queue is lower, do nothing. Else, add this node heuristic instead
          AStarNode nodeFromOpenQueue = findAStarNode(n, openQueue);
          if (!closedList.contains(n) && nodeFromOpenQueue != null ) {
            if (nodeFromOpenQueue.gValue >= gValue) {
              //Updating value, must remove and reinsert element so the priority is updated
              openQueue.remove(nodeFromOpenQueue);
              nodeFromOpenQueue.fValue = fValue;
              nodeFromOpenQueue.gValue = gValue;
              nodeFromOpenQueue.visitedThrough = current;

              openQueue.add(nodeFromOpenQueue);
            }
          } else {
            openQueue.add(new AStarNode(n, fValue, gValue, current));
          }
        }
      }
    }
  }

  // Used to optimize the time it takes to travel the shortest path home
  Direction getDirection(Node a, Node b) {
    if (a.x == b.x && a.y > b.y) {
      return Direction.NORTH;
    } else if (a.x < b.x && a.y > b.y) {
      return Direction.NORTHEAST;
    } else if (a.x < b.x && a.y == b.y) {
      return Direction.EAST;
    } else if (a.x < b.x && a.y < b.y) {
      return Direction.SOUTHEAST;
    } else if (a.x == b.x && a.y < b.y) {
      return Direction.SOUTH;
    } else if (a.x > b.x && a.y < b.y) {
      return Direction.SOUTHWEST;
    } else if (a.x > b.x && a.y == b.y) {
      return Direction.WEST;
    } else {
      return Direction.NORTHWEST;
    }
  }

  // Returns the AStarNode in queue containing the Node node.
  // Returns null if no such AStarNode exists.
  AStarNode findAStarNode(Node node, Queue<AStarNode> queue) {
    for (AStarNode n : queue) {
      if (n.node.equals(node))
        return n;
    }
    return null;
  }

  // Moves to the next node in the pathHome list that we got from findShortestPathHome
  void takeOneMoreStepHome() {

    if (!pathHome.isEmpty()) {
      Node next = pathHome.poll();
      okayToGoNextStepHome = false;

      if (pathHome.isEmpty()) {

        //This fun part is used to check how much further we need to move in x and y direction to get into the homebase
        int x = 0, y = 0;
        if (lastDir == Direction.NORTH || lastDir == Direction.NORTHWEST || lastDir == Direction.NORTHEAST) {
          y = 50;
        } else if (lastDir == Direction.SOUTH || lastDir == Direction.SOUTHWEST || lastDir == Direction.SOUTHEAST) {
          y = -50;
        } 

        if (lastDir == Direction.EAST || lastDir == Direction.NORTHEAST || lastDir == Direction.SOUTHEAST) {
          x = -50;
        } else if (lastDir == Direction.WEST || lastDir == Direction.SOUTHWEST || lastDir == Direction.NORTHWEST) {
          x = 50;
        } 

        float a = next.x+x;
        float b = next.y+y;
        currentNode = next;
        moveTo(next.x+x, next.y+y);
      } else {
        moveTo(next.x, next.y);
      }
    }
  }  

  // Check if the Node current is within homebase
  boolean isNodeInHomeBase(Node current) {
    if (
      current.x > team.homebase_x && 
      current.x <= team.homebase_x+team.homebase_width &&
      current.y > team.homebase_y &&
      current.y <= team.homebase_y+team.homebase_height) {
      return true;
    } else {
      return false;
    }
  }

  // real distance from start to currentNode
  double calculateGValue(AStarNode a, Node b) {
    double prevDist = a.gValue;
    double newDist = Math.sqrt(Math.pow(a.node.x-b.x, 2)+Math.pow(a.node.y-b.y, 2));
    return prevDist + newDist;
  }

  double calculateHeuristics(Node n) {
    //If we think of the game plan as a coordinate system where the point (team.homebase_x+team.homebase_width, team.homebase_y+team.homebase_height) is origo,
    //then the following if-statements determines whether the Node n is in the first, third or fourth quadrant and calculates accordingly
    if (n.x <= team.homebase_x+team.homebase_width) { // n is in the third quadrant
      return n.y-(team.homebase_y+team.homebase_height);
    } else if (n.y <= team.homebase_y+team.homebase_height) { // n is in the first quadrant
      return n.x-(team.homebase_x+team.homebase_width);
    } else { // n is in the fourth quadrant; use euclidean distance to calculate distance to "origo"
      return Math.sqrt(Math.pow((team.homebase_x+team.homebase_width)-n.x, 2)+Math.pow((team.homebase_y+team.homebase_height)-n.y, 2));
    }
  }

  void flock() {
    ArrayList<Tank> flock = new ArrayList(Arrays.asList(team.tanks));
    flock.remove(this);

    PVector sep = separate(flock);
    PVector ali = align(flock);
    PVector coh = cohesion(flock);
    PVector hea = new PVector(cos(team.getHeading()), sin(team.getHeading()));
    sep.mult(15.0);
    ali.mult(1.0);
    coh.mult(1.0);
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
    //applyForce(hea);
    
  }

  PVector align(ArrayList<Tank> tanks) {
    float dist = 50;

    PVector sum = new PVector();
    int count = 0;
    for (Tank other : tanks) {
      float d = PVector.dist(position, other.position);
      if((d > 0) && (d < dist)){
      sum.add(other.velocity);
      count++;
      }
    }
    if(count > 0){
    sum.div(count);
    sum.setMag(maxspeed);
    PVector steer = PVector.sub(sum, velocity);
    steer.limit(maxforce);
    return steer;
    }
    return new PVector();
  }


  PVector separate(ArrayList<Tank> tanks) {
    float desiredseparation = 65.0f; // TODO: bra värde??
    PVector steer = new PVector(0,0,0);
    int count = 0;
    for (Tank other : tanks) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < desiredseparation)) {
        PVector diff = PVector.sub(position,other.position);
        diff.normalize();
        diff.div(d);
        steer.add(diff);
        count++;
      }
    }
    if(count > 0)
      steer.div((float)tanks.size());
      
        // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  PVector cohesion(ArrayList<Tank> tanks) {
    float dist = 50;
    int count = 0;
    PVector sum = new PVector(0,0);
    for (Tank other : tanks) {
      float d = PVector.dist(position, other.position);
      if((d > 0) && (d < dist)){
      sum.add(other.position);
      count++;
      }
    }
    if(count > 0) {
    sum.div(count);
    sum.sub(position);
    return seek(sum);
    }
    return new PVector();
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);
    desired.setMag(maxspeed);
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);
    return steer;
  }


  // container used for the A* algorithm
  class AStarNode {
    Node node;
    double fValue;
    double gValue;
    AStarNode visitedThrough;

    AStarNode(Node node, double fValue, double gValue, AStarNode visitedThrough) {
      this.node = node;
      this.fValue = fValue;
      this.gValue = gValue;
      this.visitedThrough = visitedThrough;
    }
    @Override
      public String toString() {
      return "(" + node.col +  ", "+ node.row  + ")" + " + " + "(" +fValue+ "), ";
    }
  }

  class HeuristicsComparator implements Comparator<AStarNode> {
    @Override
      public int compare(AStarNode n1, AStarNode n2) {
      return n1.fValue > n2.fValue ? 1 : -1;
    }
  }

  // Got from https://forum.processing.org/one/topic/timer-in-processing.html. The class is used to make sure the tank stays still for three seconds after reporting to its homebase 
  class StopWatchTimer {
    int startTime = 0, stopTime = 0;
    boolean running = false;  


    void start() {
      startTime = millis();
      running = true;
    }
    void stop() {
      stopTime = millis();
      running = false;
    }
    int getElapsedTime() {
      int elapsed;
      if (running) {
        elapsed = (millis() - startTime);
      } else {
        elapsed = (stopTime - startTime);
      }
      return elapsed;
    }
    int seconds() {
      return (getElapsedTime() / 1000) % 60;
    }
  }
  
  // Anropas från team ("hembasen") som radio
  public void addPatrolledNodeFromOtherTank(Node n) {
    patrolled.put(n, 0); 
  }
}


private enum Direction {
  SOUTH, 
    NORTH, 
    EAST, 
    WEST, 
    NORTHEAST, 
    SOUTHEAST, 
    NORTHWEST, 
    SOUTHWEST
}
