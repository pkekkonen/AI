class Team {

  Tank[] tanks = new Tank[3];
  int id; // team red 0, team blue 1.
  int tank_size;
  PVector tank0_startpos = new PVector();
  PVector tank1_startpos = new PVector();
  PVector tank2_startpos = new PVector();
  HashMap<Node, Integer> patrolled = new HashMap<Node, Integer>();
  float homebase_x;
  float homebase_y;
  float homebase_width = 150;
  float homebase_height = 350;

  color team_color;
  StopWatchTimer timer;
  int numberOfHits; // sammalagda antalet bekräftade träffar på andra lagets tanks. 


  Team (int team_id, int tank_size, color c, 
    PVector tank0_startpos, int tank0_id, CannonBall ball0, 
    PVector tank1_startpos, int tank1_id, CannonBall ball1, 
    PVector tank2_startpos, int tank2_id, CannonBall ball2) 
  {
    this.id = team_id;
    this.tank_size = tank_size;
    this.team_color = c;
    this.tank0_startpos.set(tank0_startpos);
    this.tank1_startpos.set(tank1_startpos);
    this.tank2_startpos.set(tank2_startpos);

    this.numberOfHits = 0; 

    tanks[0] = new Tank(tank0_id, this, this.tank0_startpos, this.tank_size, ball0);
    tanks[1] = new Tank(tank1_id, this, this.tank1_startpos, this.tank_size, ball1);
    tanks[2] = new Tank(tank2_id, this, this.tank2_startpos, this.tank_size, ball2);


    if (this.id==0) {
      this.homebase_x = 0; 
      this.homebase_y = 0;
    } else if (this.id==1) {
      this.homebase_x = width - 151; 
      this.homebase_y = height - 351;
    }
  }

  int getId() {
    return this.id;
  }

  color getColor() {
    return this.team_color;
  }

  void messageSuccessfulHit() {
    this.numberOfHits += 1;
  }

  void updateLogic() {
  }


  Node getNextTarget(Tank t) {
    Node currentNode = grid.getNearestNode(t.position);
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
    checkEnvironment(t); 
    return target;
  }
  
  void checkEnvironment(Tank t) {
    //checkEnvironment_sensor();

    // Check for collisions with Canvas Boundaries
    float r = t.diameter/2;
    if ((t.position.y+r > height) || (t.position.y-r < 0) ||
      (t.position.x+r > width) || (t.position.x-r < 0)) {
      if (!t.stop_state) {
        t.position.set(t.positionPrev); // Flytta tillbaka.
        //println("***");
        t.stopMoving_state();
      }
    }

    if (
      t.position.x > homebase_x && 
      t.position.x < homebase_x+homebase_width &&
      t.position.y > homebase_y &&
      t.position.y < homebase_y+homebase_height) {
      if (!t.isAtHomebase) {
        t.isAtHomebase = true;

        // checks if reporting tank has just arrived to the homebase
        if (t.isReporting) {
          timer = new StopWatchTimer();
          timer.start();
          t.isReporting = false;
          t.isReportingInHomebase = true;
          patrolled.put(t.lastSeenEnemy, Integer.MAX_VALUE);
          for (Node n : getNeighboringNodes(t.lastSeenEnemy)) {
            patrolled.put(n, Integer.MAX_VALUE);
          }
        }
        t.message_arrivedAtHomebase();
      }
    } else {
      t.isAtHomebase = false;
    }
  }

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

  void backPropagate(Node n, int i) {
    for (Node temp : getNeighboringNodes(n)) {
      if (patrolled.containsKey(temp) && (patrolled.get(temp) == -1 || (patrolled.get(temp) > i+1 && patrolled.get(temp) != Integer.MAX_VALUE))) {
        patrolled.put(temp, i+1); 
        backPropagate(temp, i+1);
      }
    }
  }

  void assignCostValue(ArrayList<Node> finished, Node n) {
    ArrayList<Node> neighbors = getNeighboringNodes(n);
    float r = tank_size/2;
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
  // Används inte.
  // Hemma i homebase
  //boolean isInHomebase(PVector pos) {
  //  return true;
  //}

  void displayHomeBaseTeam() {
    strokeWeight(1);
    //fill(204, 50, 50, 15);
    fill(this.team_color, 15);
    //rect(0, 0, 150, 350);
    rect(this.homebase_x, this.homebase_y, this.homebase_width, this.homebase_height);
  }


  void displayHomeBase() {
    displayHomeBaseTeam();
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
