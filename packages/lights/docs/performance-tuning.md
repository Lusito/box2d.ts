# Performance Tuning

The following is copied from the original documentation and **might need to be adjusted** for the TypeScript version:

## Object creation
 * Ray Handler is heavy weight object. It contains many float arrays and couple FBO's when openGL es 2.0 is available. Ray Handler can be safely used over the application lifetime. Ray Handler need to be disposed when its destroyed.
 * Lights are moderate weight objects. Every light contain two mesh and if possible try to reuse these lights. Just set light to disabled if you know that you will be needing another that type of light soon. Remove lights if you don't need them anymore or there is big time window between them. Be aware that setActive(false) do not set body reference to null.

## Performance tips
 * Tips are marked with *(CPU | GPU | MEMORY)* depending which part of system that tip might help.
 * Disabled light do not have any performance overhead. Disable lights are stored on separate list. *(CPU & GPU)*
 * Disable shadows if game is running on platform with GL es 1.0 and where render target do not have alpha channel. Usually this mean android with gles1.0. This maybe can be done automatically in library but don't count on that! *(GPU)*
 * Static flag make light very cheap. You can use static light even for objects that move but they do it rarely. Static lights can be attached to bodies but they do not follow body movement after initial attach call. Every time you call method that alter static light state it has to be updated. Static lights is wrong choice if light is updated every frame. *(CPU)*
 * Xray flag prevent all the raycasting for the light. So this lower cpu load about 80%. Xrays are optimal for small objects and usually dynamic light look better/smoother when combined with xray pointlight. Use this flag allways when you don't need dynamic behavior. *(CPU)*
 * Setting FBO size to small make really big difference with performance. First it helps with light drawing because its use less fillrate. Secondly it help with blurring. One blur pass use 5+5 dynamic texture fetch. With 800x480 screen: using original screen size as fbo size and three blur pass would need over 10 million texture look ups. But using box2dLights defaults settings 200x120(quarter of screen sizes) and one blur pass use only 240 000 texture look ups but yield more smoothed look. *(GPU & MEMORY)*
 * Blur passes are heavyweight, try to smaller fbo combined with more passes if really blurred lights are needed. Idea for blur algorithm can be found here http://rastergrid.com/blog/2010/09/efficient-gaussian-blur-with-linear-sampling/ *(GPU)* 
 * For dynamic lights use only that amount rays that are absolute needed. Usually best outcome is somewhere between 5 and 128. For something really accurate and visible you sometimes need more but be careful. For every ray box2dLights have to use one raycast and 1 vertex + 2 for soft light.*(CPU & GPU & MEMORY)*
 * Light distance make light a bit heavier. Ray Cast need to travel bigger distance and more pixels need to drawn. For bigger light try first add color alpha channel and maybe add some white color. If that does not help try to use different fallof scheme(Still WIP).*(CPU & GPU)*
 * Set culling enabled. Culling is very lightweight process but safe all raycast calculations and drawing. Lights are culled automatically if they are so much off screen that even lights tips would never occur in screen. Culled light is still not free so if you know something is behind camera really long time maybe that could be disabled.*(CPU & GPU)*
 * Be reasonable with soft light distance and turn softness off it that is not needed.*(CPU & GPU)*
 * Only create rayHandler with maxNumberRays that are needed *(MEMORY)*
 * If physics are stepped more sparsely than game is rendered ray handler update can be skipped if physic did not skipped between frames. If physics are updated more often that game is rendered then calling updateAndRender is best choice.*(CPU)*
 * If not single lights are rendered blurring and light map blitting is skipped. So cull/disable lights that are not visible (GPU)