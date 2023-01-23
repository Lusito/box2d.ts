# Glossary

The following is copied from the original documentation and **might need to be adjusted** for the TypeScript version:

  * *Ray Handler* is handler class for all the lights. Ray handler manages updating, rendering and disposing the lights. Ray handler also checks the current OpenGl es version and uses the right mode for rendering. If GLes2.0 is detected: gaussian blurred lights map is enabled and used automatically. GLes2.0 has bigger constant time but it scales better with multiple lights because lights are rendered to small FBO instead of render target and the fragment shader has to do less work.
    * `new RayHandler(box2dWorld, fboWidth, fboHeight);`

## Light types
  * *Point Light* is first concrete class. It's the simplest and most used light. Point lights have meaningful position and distance and all other lights attributes. Point lights are always circular shaped.
    * `new PointLight(rayHandler, numRays, color, maxDistance, x, y);`
  * *Cone Light* is second concrete class. It's basically a point light but only a sector of the full circle. Cone lights have direction and cone degree as additional parameters. cone degree is aberration of straight line to both directions. Setting Cone Degree to 180 means that you got full circle.
    * `new ConeLight(rayHandler, numRays, color, maxDistance, x, y, directionDegree, coneDegree);`
  * *Directional Light* simulate light source that location is at infinite distance. This means that direction and intensity is the same everywhere. -90 direction is straight from up. This type of light is good for simulating the sun.
    * `new DirectionalLight(rayHandler, numRays, color, directionDegree);`

## Light abstract classes
  * *Light* Abstract class that contain shared parameters and act as "interface" for all the lights. Light is made from a bunch of rays that are tested with raycasting against box2d geometry and from this data the light mesh is constructed and rendered to scene. All lights have meaningful color, number of rays, softShadowLenght and some booleans like active, soft, xray, staticLight.
  * *Positional Light* is also an abstract class. This contain shared parameters between Point- and Cone light. Positional light have position and finite distance.