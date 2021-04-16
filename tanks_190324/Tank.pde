import java.util.Comparator; //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>//
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.List;
import java.util.LinkedList;

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
  ArrayList<Node> visited = new ArrayList<Node>(); // lista över de noder som traverserats, inkl backtracking
  Node lastVisited; // den senaste noden som tanksen var på
  boolean turningBack; // Behöver tanksen backa pga collision eller att det är dags att backtracka? This is the boolean for you
  ArrayList<Node> badSpace = new ArrayList<Node>(); // har en nod fått din tanks att backa? Mark it with this
  int counter; //Hur pass mycket noden behöver backa
  private ArrayList<Node> enemyNodes = new ArrayList<Node>(); //lista av noder i fiendebas

  Node startNode; // noden där tanken befinner sig.

  boolean hasTarget; // Används just nu för att kunna köra "manuellt" (ai har normalt target).
  PVector targetPosition; // Används vid förflyttning mot target.
  float targetHeading; // Används vid rotation mot en target.
  PVector sensor_targetPosition;

  PVector[] otherTanks  = new PVector[5];
  PVector distance3_sensor;
  boolean tankAhead; //Används för att kontrollera ifall det är en tank ahead. 

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
  boolean patrolling; //starts the patrolling, called in tanks_190304
  boolean isMovingOnPatroll; //Tanksen är ute och patrullerar

  boolean idle_state; // Kan användas när tanken inte har nåt att göra.

  boolean isMoving; // Tanken är i rörelse.
  boolean isRotating; // Tanken håller på att rotera.
  boolean isColliding; // Tanken håller på att krocka.
  boolean isAtHomebase;
  boolean userControlled; // Om användaren har tagit över kontrollen.
  boolean goingHome;
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
    visited.add(startNode); //Lägger till startpositionen till listan av noder som traversats
    println("visited "+visited);
    println("Startnode "+startNode.col +" " + startNode.row);
    startNode.setVisited(true); // sätter startnodens variable visited till true
    this.lastVisited = grid.getNearestNode(this.startpos); //Lägger till startpositionen som senast traverserade nod
    this.counter = 1; //sätter counter till ett, då listan börjar med att backa ett steg.
    this.tankAhead = false; //Startar inte med en tank framför oss vad vi vet

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
    this.patrolling = false; //Patrolling startar från tanks_190304
    this.isMovingOnPatroll = false; // Patrolling startar från tanks_190304

    // Under test
    this.isMoving = false;
    this.isRotating = false;
    this.isAtHomebase = true;
    this.idle_state = true;
    this.goingHome = false;
    this.okayToGoNextStepHome = true;

    this.ball = ball;
    this.hasShot = false;
    this.maxspeed = 3; //3;
    this.maxforce = 0.1;
    this.maxrotationspeed = radians(3);
    this.rotation_speed = 0;
    this.image_scale = 0.5;
    this.isColliding = false;
    this.turningBack = false; //Man startar generellt inte med att backa


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


    initializeSensors();
    
    Node[][] nodes = grid.getAllNodes();
    for (int i = 0; i < nodes.length; i++) {
      for (int j = 0; j < nodes[i].length; j++) {
        if (nodes[i][j].x > width-150 && nodes[i][j].x < width && nodes[i][j].y > height-300 && nodes[i][j].y < height) {
          enemyNodes.add(nodes[i][j]);
        }
      }
    }
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
    println("*** Tank["+ this.getId() + "].moveTo(float x, float y)");

    moveTo(new PVector(x, y));
  }

  //**************************************************
  void moveTo(PVector coord) {
    //println("*** Tank["+ this.getId() + "].moveTo(PVector)");
    if (!isImmobilized) {
      println("*** Tank["+ this.getId() + "].moveTo(PVector) " + (coord.x/50-1) + " " + (coord.y/50-1));

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

    //resetTargetStates();
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


// Ändrat så baserat på huruvida tanksen ska backa ett eller flera steg eller inga alls
// väljs olika arrivedfunktioner
    if (d < 1) {
      if(turningBack && !isColliding) {
        arrivedBack();
      } else if (turningBack && isColliding) {
        arrivedBackFromCollision();
      } else {
        arrived();
      }
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
    startNode = grid.getNearestNode(this.position);  // Startnode är nu den nuvarande positionen
    lastVisited = startNode; //LastVisited är nu också den nuvarande positionen. 
    visited.add(startNode); // Lägger till nuvarande positionen över lista över besökta platser
    startNode.setVisited(true); // Ändrar variabel av nuvarande noden till visited. 
    counter = 1; // ifall counter har ändrats sätts den åter till ett, 
    //dens syfte är att kontrollera hur långt bak tanksen ska backa och det ställs till utgångsvärdet då tanksen inte behöver backa, utan kan gå normalt
    this.isMoving = false;
    isMovingOnPatroll = false;
	okayToGoNextStepHome = true; //TODO: kolla om det här behövs på andra arrived()
    if(isAtHomebase)
        goingHome = false;
    stopMoving_state();
  }
  
  //**************************************************
  // arrivedBack() gör saker lite annorlunda än arrived()
  //Den startar backtrackandet om tanksen finner sig i en situation där alla neighbouring noder redan är undersökta.
  void arrivedBack() {
    println("*** Tank["+ this.getId() + "].arrivedBack()");
    lastVisited = visited.get(visited.size()-counter); //Sätter lastVisited till senaste noden innan den fastnade
    startNode = grid.getNearestNode(this.position); //startNoden sätts i nuvarande position
    visited.add(startNode); // nuvarande position läggs till undersökt
    counter = counter + 2; // counter lägger till två för att komma förbi den nod som just lagts till och den tanksen just backat till. 
    turningBack = false; //klar med att backa
    this.isMoving = false;  
    isMovingOnPatroll = false;
    okayToGoNextStepHome = true; //TODO: kolla om det här behövs på andra arrived()
    if(isAtHomebase)
        goingHome = false;
    stopMoving_state();
  }
  
    //**************************************************
    // Denna funktion startas upp om tanksen kolliderat och nu ska tillbaka till sin tidigare plats (den nod den var på innan kollisionen)
  void arrivedBackFromCollision() {
    println("*** Tank["+ this.getId() + "].arrivedBackFromColliding()");
    lastVisited = visited.get(visited.size()-1); // lastVisited sätts till senaste värdet i visited, dvs där den var innan den krockade
    startNode = grid.getNearestNode(this.position); //Startnod sätts till nuvarande position
    visited.add(startNode); // Startnode läggs på nytt till i visited. 
    turningBack = false;
    isColliding = false;
    this.isMoving = false;  
    isMovingOnPatroll = false;
    okayToGoNextStepHome = true; //TODO: kolla om det här behövs på andra arrived()
    if(isAtHomebase)
        goingHome = false;
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
          
          //TODO: ta bort if-sats! Används endast för att testa A*
          if (goingHome) {
            takePath();
          }
          /*
          //TODO: ta bort if-sats! Används endast för att testa A*
          if (!isAtHomebase && !goingHome) {
            System.out.println("GO HOME");
            findShortestPathHome();
          }*/
          
          // Om tanken är i rörelse.
          if (this.isMoving) {
            this.heading = this.velocity.heading();
            arrive();
          }

        //Tillagt så den fortsätter patrullera om den startats och även efter att den krockat
          if ((patrolling && !isMovingOnPatroll) || isColliding) {
            keepPatrolling();
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
        if (distanceVectMag < minDistance) {
          println("! Tank["+ this.getId() + "] – FAST I ETT TRÄD");
        }
        isColliding = true; //lagt till att tanksen krockar
        badSpace.add(grid.getNearestNode(targetPosition)); // lagt till noden tanksen krockat med. 
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

      this.position.set(this.positionPrev); // Flytta tillbaka.
      if (!this.stop_state) {
        //this.position.set(this.positionPrev); // Flytta tillbaka.

        // Kontroll om att tanken inte "fastnat" i en annan tank. 
        distanceVect = PVector.sub(other.position, this.position);
        distanceVectMag = distanceVect.mag();


        if (distanceVectMag <= minDistance) {
          println("! Tank["+ this.getId() + "] – FAST I EN ANNAN TANK");
        }
        
        isColliding = true; //Lagt till att tanksen håller på att krocka
        badSpace.add(grid.getNearestNode(targetPosition)); //lägger till noden i listan över platser tanksen krockat på
        this.isMoving = false;  
        stopMoving_state();
      }

      if (this.hasShot) {
        this.ball.updateLoadedPosition(this.positionPrev);
      }


      // Meddela tanken om att kollision med den andra tanken gjorts.
      message_collision(other);
    }
    if (other.id == 0 && !tankAhead &&
    (this.id == 3 || this.id == 4 || this.id == 5)) 
      checkTankForward(this, other);
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

//*******************************************
//Kontrollerar ifall det är en fiendetank framför
  void checkTankForward(Tank other, Tank me) {
    if (!enemyNodes.contains(me.startNode)) {
      return;
    }
    PVector viewForward = PVector.add(me.position, new PVector((float)Math.cos(me.heading), (float)Math.sin(me.heading)));
    
    PVector distanceVect = PVector.sub(other.position, viewForward); 
    float distanceVectMag = distanceVect.mag(); 
    float minDistance = this.radius + other.radius; 
    if (distanceVectMag <= minDistance) {
      me.tankAhead = true; 
      println("visited "+me.visited);
      pause = true;
    }
    
    
  }
  //*****************************************
  // keepPatrolling() är den funktion som bestämmer vilken node tanksen ska till härnäst
  void keepPatrolling() {
    isMovingOnPatroll = true;

    //println("visited: "+visited);

    ArrayList<Node> neighbours_temp = grid.getNodesNeighbours(startNode); //de åtta närliggande noderna
    ArrayList<Node> neighbours = removeVisited(neighbours_temp); // de som är kvar efter noderna som a) redan är besökta och b) är inskrivna som krock-noder
    if ((neighbours.isEmpty() || isColliding) && !tankAhead) { // om listan är tom eller tanksen just krockat ska tanksen backtracka till den senaste noden den var vid. 
      Node back = new Node(lastVisited.col, lastVisited.row, lastVisited.x, lastVisited.y);
      println("back  " +back.col + " and " + back.row);
      turningBack = true;
      moveTo(back.position);
      println("last visited  " +lastVisited.col + " and " + lastVisited.row);
    }else if (tankAhead) {
      findShortestPathHome();
    } else { // om grannlistan inte är tom, ska en random väljas ut och åkas till. 
      Node target = grid.getRandomNodeWithin(neighbours);
      println("target  " +target.col + " and " + target.row);
      moveTo(target.position);
      println("last visited  " +lastVisited.col + " and " + lastVisited.row);
      //}
    }
  }


//*****************************************************
// removeVisited tar in listan med ens nods närmsta noder och sorterar bort de som 
// redan är besökta och redan är krockade med. 
  ArrayList<Node> removeVisited(ArrayList<Node> list) {
    ArrayList<Node> removed = new ArrayList<Node>();

    for (Node n : list) {
      if (n.getVisited() == false && !(badSpace.contains(n))) {
        removed.add(n);
        println("removed: " + n.col + "  " + n.row);
      }
    }
    return removed;
  }

  //*****************************

  //TODO: make better implementation
  Direction lastDir;

  // using A* f(n) = g(n) + h(n)
  void findShortestPathHome() {
    println("Find shortest path home");
    Queue<AStarNode> openQueue = new PriorityQueue<AStarNode>(new HeuristicsComparator());
    LinkedList<AStarNode> closedList = new LinkedList<AStarNode>(); 
    openQueue.add(new AStarNode(this.startNode, calculateHeuristics(this.startNode), 0, null)); //adding start node
    println("FINDING SHORTESt: curr " + this.startNode);
    AStarNode current;
    
    while (!openQueue.isEmpty()) {
      current = openQueue.poll();
      closedList.add(current);


      if (isNodeInHomeBase(current.node)) {

        //WE ARE DONE
        LinkedList<Node> finalPath = new LinkedList<Node>();
        LinkedList<Node> actualFinalPath = new LinkedList<Node>();
        AStarNode currNode = closedList.getLast();
        lastDir = getDirection(currNode.node, currNode.visitedThrough.node);

        Direction dir = null;
        Direction prevDir = null;
        
        while(currNode != null) {
          prevDir = dir;
          if(currNode.visitedThrough != null) {
            dir = getDirection(currNode.node, currNode.visitedThrough.node);
          }
          if(dir == null ||  dir != prevDir) { 
            finalPath.addFirst(currNode.node);             
          }
          actualFinalPath.addFirst(currNode.node);               
          currNode = currNode.visitedThrough;

        }
        System.out.println(actualFinalPath);
        System.out.println(finalPath);
        pathHome = finalPath;
        goingHome = true;

        return;
      }

      // få närliggande som vi känner till från current 
      // ska ligga i patrolled
      List<Node> children = grid.getNodesNeighbours(current.node);

      for (Node n : children) {
        if (visited.contains(n)) {
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
  
  Direction getDirection(Node a, Node b) {
    if(a.x == b.x && a.y > b.y) {
      return Direction.NORTH;
    } else if(a.x < b.x && a.y > b.y) {
      return Direction.NORTHEAST;
    } else if(a.x < b.x && a.y == b.y) {
      return Direction.EAST;
    } else if(a.x < b.x && a.y < b.y) {
      return Direction.SOUTHEAST;
    } else if(a.x == b.x && a.y < b.y) {
      return Direction.SOUTH;
    } else if(a.x > b.x && a.y < b.y) {
      return Direction.SOUTHWEST;
    } else if(a.x > b.x && a.y == b.y) {
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

  // 
  void takePath() {

    if (okayToGoNextStepHome && !pathHome.isEmpty()) {
      Node next = pathHome.poll();
      okayToGoNextStepHome = false;

      if (pathHome.isEmpty()) {
        //TODO: make better implementation
        int x = 0, y = 0;
        if(lastDir == Direction.NORTH || lastDir == Direction.NORTHWEST || lastDir == Direction.NORTHEAST) {
          y = 1;
        } else if(lastDir == Direction.SOUTH || lastDir == Direction.SOUTHWEST || lastDir == Direction.SOUTHEAST) {
          y = -1;
        } 
        
        if(lastDir == Direction.EAST || lastDir == Direction.NORTHEAST || lastDir == Direction.SOUTHEAST) {
          x = -1;
        } else if(lastDir == Direction.WEST || lastDir == Direction.SOUTHWEST || lastDir == Direction.NORTHWEST) {
          x = 1;
        } 
        
        goingHome = false;
        float a = next.x+x;
        float b = next.y+y;
        println("final MOVE: " + a +", "+ b );
        moveTo(next.x+x, next.y+y);
      } else {
          moveTo(next.x, next.y);
      }
      
    }
  }  

  // Check if node is within homebase
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

  // real distance from start to current
  double calculateGValue(AStarNode a, Node b) {
    double prevDist = a.gValue;
    double newDist = Math.sqrt(Math.pow(a.node.x-b.x, 2)+Math.pow(a.node.y-b.y, 2));
    return prevDist + newDist;
  }

  double calculateHeuristics(Node n) {
    
    //If we think of the game plan as a coordinate system where the point (team.homebase_x+team.homebase_width, team.homebase_y+team.homebase_height) is origo,
    //then the following if-statements determines whether the Node n is in the first, third or fourth quadrant and calculates accordingly
    if(n.x <= team.homebase_x+team.homebase_width) { // n is in the third quadrant
      return n.y-(team.homebase_y+team.homebase_height);
    } else if(n.y <= team.homebase_y+team.homebase_height) { // n is in the first quadrant
      return n.x-(team.homebase_x+team.homebase_width);
    } else { // n is in the fourth quadrant; use euclidean distance to calculate distance to "origo"
      return Math.sqrt(Math.pow((team.homebase_x+team.homebase_width)-n.x, 2)+Math.pow((team.homebase_y+team.homebase_height)-n.y, 2));
    }
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

  //Static?
  class HeuristicsComparator implements Comparator<AStarNode> {
    @Override
      public int compare(AStarNode n1, AStarNode n2) {
      return n1.fValue > n2.fValue ? 1 : -1;
    }
  }

//  //TODO: ta bort metod! Används endast för att testa A*
//  void testAStarAlgorithmByAddingPatrolledNodes() {
//    //SHORTEST
//    //patrolled.put(grid.nodes[2][6], 0);
//    patrolled.put(grid.nodes[2][7], 0);
//    patrolled.put(grid.nodes[2][8], 0); // but should skip this one
//    patrolled.put(grid.nodes[3][8], 0);
//    patrolled.put(grid.nodes[4][8], 0);
//    patrolled.put(grid.nodes[5][9], 0);
//    patrolled.put(grid.nodes[6][10], 0);


//    //Longer path
//    patrolled.put(grid.nodes[6][9], 0);
//    patrolled.put(grid.nodes[6][8], 0);
//    patrolled.put(grid.nodes[6][7], 0);
//    patrolled.put(grid.nodes[6][6], 0);
//    //patrolled.put(grid.nodes[5][6], 0);
//    patrolled.put(grid.nodes[4][6], 0);
//    patrolled.put(grid.nodes[3][6], 0);
    
    
//    patrolled.put(grid.nodes[6][11], 0);
//    patrolled.put(grid.nodes[6][12], 0);
//    patrolled.put(grid.nodes[6][13], 0);
//    patrolled.put(grid.nodes[6][14], 0);
//    patrolled.put(grid.nodes[5][14], 0);
//    patrolled.put(grid.nodes[4][14], 0);
//    patrolled.put(grid.nodes[3][14], 0);
//    patrolled.put(grid.nodes[2][14], 0);
//    patrolled.put(grid.nodes[1][14], 0);
//    patrolled.put(grid.nodes[0][13], 0);
//    patrolled.put(grid.nodes[0][12], 0);
//    patrolled.put(grid.nodes[0][11], 0);
//    patrolled.put(grid.nodes[0][10], 0);
//    patrolled.put(grid.nodes[0][9], 0);
//    patrolled.put(grid.nodes[0][8], 0);
//    patrolled.put(grid.nodes[0][7], 0);
//    patrolled.put(grid.nodes[0][6], 0);
//    patrolled.put(grid.nodes[0][5], 0);
//    patrolled.put(grid.nodes[0][4], 0);
//}


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
        }
        else {
            elapsed = (stopTime - startTime);
        }
        return elapsed;
    }
    int seconds() {
      return (getElapsedTime() / 1000) % 60;
    }
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
