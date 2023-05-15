# A set of processes talking to VISCA camera.

Borrowed some code from
https://github.com/mutax/PyVisca/blob/master/pyviscalib/visca.py

## Architecture

* A single `driver` process directly controls the camera.
* Keyboard strokes are consumed by a separate process communicating with the
`driver` via named pipes.
* Similarly REST service (flask) consumes commands which it communicates to the
`driver` via named pipes.


```
[   Driver                                                  ]
  ^                      |                        |
  | /tmp/visca-driver    | /tmp/visca-keyboard    |
  |                      V                        | /tmp/visca-rest
  +--------------------[visca-keyboard]           |
  |                                               V
  +--------------------------------------------[[  flask ]]



```
