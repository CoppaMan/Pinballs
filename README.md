# Physically-based Simulation 2018 - Project of Pinballs

## Disclaimer
This project is built upon the exercise framework of the Physically-based Simulation course and uses the SFML library to interface keyboard and speakers. Every other functinality mentioned in the next section was implemented by Simon Huber and Dario Morandini.

## Implemented functionality

### Effects
The Effect class presents a general interface to change the behaviour of colliding objects. It is called in **CollsionDetection::applyImpulse()** and can be updated frame by frame in the **PinballSim::advance()** simulation loop. This allows for not only modifications during the collision frame but also during a short period after the initial collision. Effects also have a cooldown which describes a time during which the same effect cannot be triggered again. Effects know of their collision partner and can also change their properties.

- **GravityEffect**: Changes the gravity of the play field
- **ScoreEffect**: Updates the score of the current game (for Score see [here](#score))
- **SoundEffect**: Plays a sound clip after the collision, the cooldown timer makes sure to no play it too often in succession.
- **ColorEffect**: Uses the update function to create either a linear or constant color fade effect.
- **ForceEffect**: Does not change the linear momentum of the object attached to it but rather of its collision partner. Used for the launch of the ball

### Score
The score is represented by both a natural number as well as an array of 7 segment displays made up of rigid bodies. The score consists of a variable number of **Digit**s. When **addScore()** is called it performs modular arithmetics to partition the decimal number into its digits. To prevent overflows the number of digits limits the highest score possible.

- **Segement**: A segment is a **RigidBody** with additional attributes to enable it (make it red) or disable it (make it black). It uses a new **ObjType** called **INTANGIBLE** to not check for collsions.
- **Digit**: A digit is made up of 7 segments. Its main objective is to enable and disable the correct **segment**s to represent one decimal which is performed in **update_digit()**.

### Obstacles
Placing **RigidObject**s on a table which has rotation different from the identity matrix is cumbersome on its own. The **Obstacle** class takes a parent (in our case the **Table**) and orients the **RigidObject**s accordingly. Multiple **RigidObject**s can be added to one **Obstacle** by naming the meshes *obstacleName_number* where number starts a 0. Those meshes need to share a common origin.

### Paddle
The paddle is a **RigidObject** which can be activated by a specified key press. If the key is not pressed, the paddle will return to its resting configuration. Like **Obstacle**s it is placed relative to the table. To catch the key event produced by SFML, a hook is placed in **PinballSim::advance()** with the **toggle()** method. By setting its angular momentum we can handle a **Paddle** collision like any other collision.