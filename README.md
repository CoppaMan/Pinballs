# Physically-based Simulation 2018 - Project of Pinballs

## Disclaimer
This project is built upon the exercise framework of the Physically-based Simulation course and uses the SFML library to interface keyboard and speakers. Every other functinality mentioned in the next sections were implemented by Simon Huber and Dario Morandini.

## Compiling the project
The project can be compiled with the same directives as for the exercide framework. One additional requirement is the dynamic library SFML.


## Project Presentations
- Project Introduction: https://docs.google.com/presentation/d/13hZBWNBJWIUGUl8o-nl4So_00HFmScay813Xa9ILxs8/edit?usp=sharing
- Milestones: https://docs.google.com/presentation/d/1g-YJ9eIOaLUTn--ezqx5tXPgiCXUQ7etN0Hz8Ovv49Y/edit?usp=sharing
- Final: https://docs.google.com/presentation/d/1y-D4_HHPXiFdv9KLpba9SAw5Yb1Uuq2BKoNT4LCxIUQ/edit?usp=sharing
Also available locally on the root of this repo

## Demonstration video
- One ball + sounds: https://www.youtube.com/watch?v=opUgCQ8sEeQ&feature=youtu.be
- Multiple balls: https://www.youtube.com/watch?v=21iKxu6ihqk



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

### Collision Detection 

- Broad collision: The rigid object has new a getBoundingType() where we have 2 possible bounding types, spheres and bounding boxes. 
- Narrow collision:
    Algorithm outline (for each objects pair from AABB): 
        1. Assert that one of the objects is a dynamic object. 
        2. If one object is the table surface use specific table collision
        3. Else use GJK + EPA (gjk2.h)

gjk2.h this header has 2 methods. Either boolean run(Shape &A, Shape &B, Contact &c) which fills the contact data if returns true and if false there is no collision. Or runWithCCD(Shape &A, Shape &B, Contact &c, double timeDelta) which does the same just with continuous collision detection. (Simple binary search sceme) 



### Continuous Collision Detection 
Uses multiple binary search passes to find an interpolated position of two colliding objects which represents the first collision frame. We then perform EPA on this frame resulting in a more accurate penetration depth and normal. It can also be used to backtrack along the velocity vector of both objects to position them before the contact. This can be done using the ratio member of the contact struct which represents by how much of the last velocity vector is not colliding.