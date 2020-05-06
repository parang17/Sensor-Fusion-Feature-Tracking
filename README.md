# Camera Feature Tracking
Main objective of this project is developing camera feature tracking algorithm and test different combination of the detector/extractor with respect to computational time and their performance.
<img src="images/keypoints.png" width="820" height="248" />

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

The following result is the summary of computational time and its performance.
## Keypoint Counting
Table 1. Keypoint Counting for keypoint detection
| Detector |Img 0|Img 1|Img 2|Img 3|Img 4|Img 5|Img 6|Img 7|Img 8|Img 9|Average|
| ---      | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | ---   |
|Shi-Tomasi| 125 | 118 | 123 | 120 | 120 | 113 | 114 | 123 | 111 | 112 |  118  |
| Harris   | 10  | 13  | 17  | 18  | 20  | 19  | 17  | 26  | 22  | 19  |  25   |
| FAST     | 149 | 152 | 150 | 155 | 149 | 149 | 156 | 150 | 138 | 143 |  149  |
| BRISK    | 264 | 282 | 282 | 277 | 297 | 279 | 289 | 272 | 266 | 254 |  276  |
| ORB      | 92  | 102 | 106 | 113 | 109 | 125 | 130 | 129 | 127 | 128 |  116  |
| AKAZE    | 166 | 157 | 161 | 155 | 163 | 164 | 173 | 175 | 177 | 179 |  167  |
| SIFT     | 138 | 132 | 124 | 137 | 134 | 140 | 137 | 148 | 159 | 137 |  138  |

## Computational time
Table 2. Computation time for keypoint detection
| Detector |Img 0    |Img 1    |Img 2    |Img 3    |Img 4    |Img 5    |Img 6    |Img 7    |Img 8    |Img 9    |Average  |
| ---      | ----    | ---     | ---     | ---     | ---     | ---     | ---     | ---     | ---     | ---     |  ---    | 
|Shi-Tomasi| 10.9544 | 8.56997 | 7.37961 | 7.24831 | 10.2637 | 10.2463 | 10.0889 | 10.2601 | 10.9104 | 11.5243 | 11.397  |
| Harris   | 12.0134 | 9.95949 | 9.81575 | 9.8395  | 9.92504 | 9.89694 | 9.90338 | 10.2937 | 11.7925 | 10.8011 | 9.21727 |
| FAST     | 0.859798| 0.795851| 0.793894| 0.784092| 0.784212| 0.781013| 0.790939| 0.781413| 0.793985| 0.791369| 0.709677|
| BRISK    | 308.082 | 290.313 | 295.705 | 303.949 | 297.694 | 283.345 | 282.146 | 283.088 | 282.643 | 283.226 | 260.211 |
| ORB      | 10.8952 | 6.91602 | 5.71017 | 4.96184 | 9.49356 | 5.6917  | 5.61432 | 6.97364 | 5.35128 | 5.0204  | 5.57329 |
| AKAZE    | 49.8981 | 46.767  | 43.2271 | 55.5888 | 49.5678 | 47.7624 | 43.9992 | 47.2797 | 42.0457 | 42.598  | 41.8836 |
| SIFT     | 63.8086 | 46.306  | 53.1428 | 49.9125 | 53.296  | 60.3458 | 46.9992 | 60.5124 | 46.7726 | 50.0129 | 46.73   |

TASK MP.8
Your eighth task is to count the number of matched keypoints for all 10 images using all possible combinations of detectors
 and descriptors. In the matching step, use the BF approach with the descriptor distance ratio set to 0.8.

## Matching Statistics
Table 3. Summary of all the detection/extraction/matching 
| Combination(detect + descriptor)| # Detected Keypoints| Detection Time(ms) | Extraction Time(ms) | #Matched Keypoint | Total Time (ms)    |
| ---                             | ---                 | ---                | ---                 | ---               | ---                |                 
| Shi-Tomasi + SIFT               |      1342           |    8.61091         |     8.04911         |     106           |    16.6649         |
| Shi-Tomasi + ORB                |      1342           |    11.3529         |     29.4689         |     106           |    14.1332         |
| Shi-Tomasi + FREAK              |      1342           |    8.39176         |     50.50ms         |     106           |    37.8662         |
| Shi-Tomasi + AKAZE              |      N/A            |    N/A             |     N/A             |     N/A           |    N/A             |   
| Shi-Tomasi + BRIEF              |      1342           |    9.35105         |     0.724889        |     106           |    10.0803         |
| Harris + SIFT                   |      74             |    10.7021         |     11.0068         |     16            |    21.7106         |
| Harris + ORB                    |      74             |    9.03667         |     2.25974         |     17            |    11.2974         |
| Harris + FREAK                  |      74             |    7.74706         |     28.7675         |     18            |    36.5188         |
| Harris + AKAZE                  |      N/A            |    N/A             |     N/A             |     N/A           |    N/A             |
| Harris + BRIEF                  |      74             |    8.93021         |     0.509623        |     16            |    9.44087         |
| FAST + SIFT                     |      1787           |    0.70645         |     8.6686          |     134           |    9.38071         | 
| FAST + ORB                      |      1787           |    0.714392        |     2.36592         |     134           |    3.08505         |
| FAST + FREAK                    |      1787           |    0.71246         |     28.5254         |     134           |    29.2434         |
| FAST + AKAZE                    |      N/A            |    N/A             |     N/A             |     N/A           |    N/A             |
| FAST + BRIEF                    |      1787           |    0.726779        |     0.625742        |     134           |    1.35721         |
| BRISK + SIFT                    |      2711           |    259.093         |     13.1933         |     250           |    272.298         |
| BRISK + ORB                     |      2711           |    262.21          |     8.23148         |     250           |    270.45          |
| BRISK + FREAK                   |      2711           |    260.983         |     28.9586         |     232           |    289.95          |
| BRISK + AKAZE                   |      N/A            |    N/A             |     N/A             |     N/A           |    N/A             |
| BRISK + BRIEF                   |      2711           |    258.671         |     0.672607        |     250           |    259.352         |              
| ORB + SIFT                      |      500            |    4.59901         |     15.2828         |     103           |    19.8869         |
| ORB + ORB                       |      500            |    4.61503         |     8.488           |     103           |    13.1074         | 
| ORB + FREAK                     |      500            |    4.54459         |     27.8967         |     54            |    32.445          |
| ORB + AKAZE                     |      N/A            |    N/A             |     N/A             |     N/A           |    N/A             |
| ORB + BRIEF                     |      500            |    4.54513         |     0.308792        |     103           |    4.8579          |
| AKAZE + SIFT                    |      1343           |    41.4851         |     11.623          |     149           |    53.115          |
| AKAZE + ORB                     |      1343           |    38.2283         |     6.11198         |     149           |    44.3461         |
| AKAZE + FREAK                   |      1343           |    38.6598         |     30.0045         |     149           |    68.6703         |
| AKAZE + AKAZE                   |      N/A            |    N/A             |     N/A             |     N/A           |    N/A             |
| AKAZE + BRIEF                   |      1343           |    37.339          |     0.526953        |     149           |    37.8711         |
| SIFT + SIFT                     |      1386           |    49.9193         |     37.5043         |     124           |    87.4295         | 
| SIFT + FREAK                    |      1386           |    56.7679         |     28.7188         |     123           |    85.492          |
| SIFT + AKAZE                    |      N/A            |    N/A             |     N/A             |     N/A           |    N/A             |
| SIFT + BRIEF                    |      1386           |    53.5654         |     0.792842        |     124           |    54.3631         |

TASK MP.9
Your ninth task is to log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet 
and based on this information you will then suggest the TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints 
on vehicles. Finally, in a short text, please justify your recommendation based on your observations and on the data you collected.

Here is the best three combination. The main reason is that three method have similar number of matching points between 100 ~ 140. In terms of the computational time, 
FAST + BRIEF shows the best performance, then FAST + ORB, and then ORB + BRIEF.
| Rank | Combination(detect + descriptor)| 
| 1    | FAST + BRIEF                       |  
| 2    | FAST + ORB                         |      
| 3    | ORB + BRIEF                        |      