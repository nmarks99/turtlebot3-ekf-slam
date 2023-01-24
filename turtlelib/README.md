# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
      1. We can copy the existing values in the Vector2D object and use then
      to compute the norm and then return a newly created Vector2D object from the function
      2. We can pass a pointer to the Vector2D object to the function and modify the 
      original object
      3. We can pass the Vector2D object by reference to const, read the information,
      and return a copy

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
      1. Is a good method since it simplest and safest, however it takes up more memory but this
      is often not an issue for our purposes on modern computers
      2. This was done a lot in C but is unsafe and typically shouldn't be done in modern C++. 
      A pro is that this can potentially be faster/lower memory usage.
      3. This is a good method 

   - Which of the methods would you implement and why?
      - I implemented the first method because it works, it's safe, and it's simple. To quote the C++
      core guidelines, "Prefer simple and conventional ways of passing information"

2. What is the difference between a class and a struct in C++?
Members of a struct are public by default while members of a class are 
private by default

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
The x and y fields are not dependent on each other and according to the guidelines:
"Use class if the class has an invariant; use struct if the data members can vary independently". Furthermore,
Transform2D can benefit from private members and according to the guidelines:
"Use class rather than struct if any member is non-public"

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
In order to avoid unintended type conversions by the compiler, you should, according to the guidelines
"By default, declare single-argument constructors explicit"

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
   - Declaring it const "This gives a more precise statement of design intent, better readability,
   more errors caught by the compiler, and sometimes more optimization opportunities."
   - It doesn't make sense to declare  Transform2D::operator*=() as const because it modifies the original object
