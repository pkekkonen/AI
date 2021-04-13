class Node {
  // A node object knows about its location in the grid 
  // as well as its size with the variables x,y,w,h
  float x,y;   // x,y location
  float w,h;   // width and height
  float angle; // angle for oscillating brightness
  
  PVector position;
  int col, row;
  
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
    this.col = _id_col;
    this.row = _id_row;
    this.x = _posx;
    this.y = _posy;
    
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
  } 

  //***************************************************  
  void addContent(Sprite s) {
    if (this.isEmpty) {
      this.content = s;  
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
  
  @Override
  public boolean equals(Object o) {
  
        // If the object is compared with itself then return true  
        if (o == this) {
            return true;
        }
  
        /* Check if o is an instance of Complex or not
          "null instanceof [type]" also returns false */
        if (!(o instanceof Node)) {
            return false;
        }
          
        // typecast o to Complex so that we can compare data members 
        Node n = (Node) o;
          
        // Compare the data members and return accordingly 
        return n.position.x == position.x && n.position.y == position.y;
    }
    
    @Override
    public int hashCode()
    {
        return ((int)x * (int)y / 13 * (int)angle * 7 * 149);
    }
}
