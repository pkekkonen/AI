class Node {
  // A node object knows about its location in the grid 
  // as well as its size with the variables x,y,w,h
  float x,y;   // x,y location
  float w,h;   // width and height
  float angle; // angle for oscillating brightness
  
  PVector position;
  int col, row;
  boolean visited; //Boolean ifall noden är undersökt eller ej
  
  Sprite content;
  boolean isEmpty;
  
  
  //***************************************************
  // Node Constructor 
  // Denna används för temporära jämförelser mellan Node objekt.
  Node(float _posx, float _posy) {
    this.position = new PVector(_posx, _posy);
  }

  //***************************************************  
  // Används vid skapande av grid
  Node(int _id_col, int _id_row, float _posx, float _posy) {
    this.position = new PVector(_posx, _posy);
    this.x = _posx;
    this.y = _posy;
    this.col = _id_col;
    this.row = _id_row;
    this.x = _posx; //lägger till x och y till sina värden
    this.y = _posy;
    visited = false; // sätter att vid skapandet är noden inte undersökt. 
    
    this.content = null;
    this.isEmpty = true;
  } 

  //***************************************************  
  Node(float tempX, float tempY, float tempW, float tempH, float tempAngle) {
    x = tempX;
    y = tempY;
    w = tempW;
    h = tempH;
    angle = tempAngle;
    visited = false;
  } 

   //*************************************************** 
   // Changing boolean visited
  void setVisited(boolean b) {
    visited = b;
  }
  
   //*************************************************** 
   // Accessing boolean visited
  boolean getVisited() {
    return visited;
  }

  //***************************************************  
  void addContent(Sprite s) {
    if (this.isEmpty) {
      this.content = s;
      this.isEmpty = false;
    }
  }

  //***************************************************
  boolean empty() {
    return this.isEmpty;
  }
  
  //***************************************************
  Sprite content() {
    return this.content;
  }
  

  //TODO: implement better?
  @Override
  public boolean equals(Object o){
    if (this == o)
        return true;
    if (o == null || !(o instanceof Node))
        return false;
    Node n = (Node) o;
    return x == n.x && y == n.y && col == n.col && row == n.row;
  }
  
  @Override
  public String toString() {
    return " (" + col +  ", "+ row  + ") ";//+" x: " + x + " y: " + y;
  }

}
