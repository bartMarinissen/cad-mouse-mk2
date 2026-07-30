# CAD Mouse MK2 - The accuracy project.

This is a fork of the CAD Mouse MK2 project by SB-OCR.
The mouse as designed by him works out of the box, and is quite amazing. Except for one part: The motion tracking.

This project is an attempt to tackle this, and to bring the highest possible degree of accuracy to this mouse.
This degree of accuracy may be overkill. But why settle for less than the best you can get out of the hardware! (Note, if you do want to settle for less, there is a [[good enough]] trick below with already functioning code)

## The motion problem.
The bowels of this project are rather heavy on the mathematics. So we'll be stating the problem of trying to find the position of the knob in technical terms. We have 3 sensors arranged in a triangle. At rest, the magnets in the knob float about 6mm above those sensors. The magnets themselves are 6x6 cylinders (really they are 2 3x6 cylinders stacked on top of each other, but that doesn't matter). These 3 sensors measure the magnetic field in 3 directions, which yields 9 values. Based on these 9 values, we want to find a best guess for the 'pose' of the knob. Pose means both the position and orientation of the knob.
This is not an obvious problem. However, there is a related problem that is solvable. We call this one the forward problem.

"Given a pose $P$ of the knob, what magnet sensor readings should we expect".

If we can answer that question, then we can easily find the quality of a guess. We will then use some standard optmization techniques (the Newton-Gauss method) to find the best guess, or at least something very close to it. (Oh, and we want to do it at least 100 times a second on slow hardware that doesn't natively support floating point numbers) This leads to the real problem, also known as the backwards problem (because it goes the other way). in Plain langues, the question is: given the 9 magnet sensor readings, which pose of the knob best matches these readings".
To be precise, the backwards problem is:

"What pose minimizes the sum of the squares of the difference between the 9 sensor readings and the predicted sensor readings"

### Solving the forward problem
Awnsering the forward problem is pretty straightforward. We need to do two things: figure out the strength and direction of the magnetic field around a magnet. And then figure out how the pose of the sensor relative to that magnet. We can then figure out the field strength and direction of the magnetic field at the sensor, and how that direction aligns with the orientation of the sensor. The second part is 'just' some simple translations and rotations. (The bookkeeping gets complicated and tedious, but its not hard).

The first part, figuring out the strength and direction of the magnetic field around a magnet is a solved physics problem. So we 'just' need to poor that physics into some code. It turns out that this close to a magnet, the shape of the magnet actually matters so you can't solve it with a simple model (i.e. a dipole model). The exact solutions involve horrible things like closed form elliptical integrals. Not only are those annoying parts of math, they are also slow to calculate.

Instead of calculating the field strength fully from scratch, we precompute the field (and reduce it from 3D to 2D by exploiting the axial symmetry) at a few known places. Then when we want to know the field at an unknown place we interpolate between these known places.

In summary: figure out the pose of the sensor relative to the magnet. Then figure out the field strenth and direction at that position based on interpolating known data. Then figure out how the orientation of the sensor alligns with the magnetic field direction.

### Solving the backwards problem.
As stated, we use standard optimization techniques.

These are well researched, and they work quite well. The fast ones need:
- An initial guess
- A quick way to calculate the error (as a 9-element vector)
- The Jacobian of the error w.r.t the pose.
- (They need the error to behave 'nicely' as a function of the guess of the knob pose)

The initial guess is easy. Either take the rest position, thats good enough. Or even better, take your last guess for the knob position. The quick way to calculate the error is literally just the forward problem. We have that solved already.

But the Jacobian of the error? What even is that? Multi-variate calculus is what it is. The handwavy explanation is that it tells you how the error vector changes if you slightly change the guessed knob pose. More handwaving doesn't make sense, so I won't try.

To get this Jacobian, and for things to behave nicely. We need to do some math against out forward problem. This means we need the Jacobian of our interpolator, and we then need to use a whole lot of chain rule, product rule, and quotient rule (and some general linearity of the derivative) to turn that Jacobian into our final Jacobian. On top of that, we would really like for our Jacobian to vary smoothly with the guessed pose. This means we want our interpolator to not just be continuous, we also want it to be smooth. Hence we quickly land at some form of cubic splines. (The exact choise of spline is still under investigation)

These solvers then iteratively improve the guess for the pose by evaluating the current guess, and then using the Jacobian to estimate how best to reduce the error by changing the guess. They need a few iterations. And if the forward problem is well behaved (has no valleys or plateaus) then they get there very quickly. We expect at most 20 iterations, and hope for a lot less

### Performance
We need to solve the backwards problem 100s of times per second.
That means we need to solve the forwards problem close to 1000s of times per second.
That means performance matters.

More on that to come later.

### Calibration
We said the forward problem is easy. But its only easy if you know the details of your knob exactly. For now I am considering the following details salient, there is fractal amounts of detail here, so we need to draw a line somewhere:

- The magnet strengths (non-negotiable, this is the only way to find the Z axis data)
- The sensor sensitivity per axis. (They report a field strength in mT, but they are of by a fixed percentage)
- The sensor base offsets: The sensors don't read zero even if there are no magnets nearby, their offset is much larger than just the earths magnetic field)
- Sensor skew: the sensor axes aren't necessarily perfectly orthogonal.
- Errors in the magnet pose, and more especially rotation. (After all, this is 3d printed and the magnets are slip-fit)
- Errors in the sensor pose. (This is much less of a worry because PCBs are quite accurate)

Actually finding this is for another section. But we need to know these things so we can correct for them.

## Current state of the project
We have things working at about 50Hz using a single core based on bicubic interpolation.
There is very little callibration, and its hardcoded for my knob.
We get a residual of 4% Using the mouse it seems perfectly fine.

The code is a mess, its actively being modified, its not documented yet, most things are up for modification.

But smooth working at 50Hz and a 4% residual without callibration is already something I'm proud of!

## AI usage
There is heavy AI assistance. A lot of it for the tedium of getting the Jacobians. A decent amount of it for helping write the code. Most code starts out as AI generated and then gets shaped down to something I understand and agree with. Less critical code gets less attention. Especially if tests or experience just shows that it works.

## Build instructions
There are none for now. It should work out if you just use the instructions from the original.

At the moment, if you run into issues I'd love to help. Because it would be great to know someone else is using this. I do intend to add documentation 'later'. That is as much of a promise as you probably think it is (not a promise).





