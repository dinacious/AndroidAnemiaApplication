package com.example.android.camera2video;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

/**
 * The AnemiaDetection class represents a collection of tools
 * that allow the user to accurately monitor all aspects of blood
 * pulsation. The monitoring of blood is done through the
 * camera (front or back) of the phone.
 *
 *
 *  <!<!<! PROPERTY OF THE UNIVERSITY OF WASHINGTON
 *            UBIQUITOUS COMPUTING LABORATORY !>!>!>
 *
 * @author William L. Li
 * @version 1.3
 * @since 8/18/2016
 *
 */

public class AnemiaDetection {

    private static final double ERROR_TOLERANCE = 2;
    private static final double LOW_PASS_SMOOTHING = 5.65;
    private static final double HIGH_PASS_SMOOTHING = 2;
    private static final double MINIMUM_DIFFERENCE = 0.05;

    private static final int DETECTOR_BUFFER_SIZE = 5;
    private static final int DATA_SIZE = 10;
    private static final int GARBAGE_FRAMES = 50;
    private static final int MAX_ERROR_ALLOTMENT = 4;
    private static final int MAPPING = 3;
    private static final int PEAK = 1;
    private static final int TROUGH = 2;
    private static final int AVERAGE = 3;
    private static final int MAX_LIGHT_COVER = 5;
    private static final int MAX_LIGHT_PARTIAL_COVER = 50;

    private boolean mAlreadyExecuted;
    private boolean mPrevPeakDetected;
    private boolean mPrevTroughDetected;
    private double mLpfOutput;
    private double mHpfOutput;
    private double mPrevLpfInput;
    private double mPeakHold;
    private double mTroughHold;
    private double mCalcDiff;
    private double mPeak;
    private double mTrough;
    private int mFrameCount;
    private int mTroughCount;
    private int mPeakCount;
    private int mCountHold;
    private int mPeakFrame;
    private int mTroughFrame;
    private int mSignalWidth;

    private int[] mConfidence;
    private ArrayList<Double> mFilteredData;
    private ArrayList<Double> mOriginalData;
    private Queue<Double> mElementHolder;


    public AnemiaDetection(int mFrameCount) {
        this.mAlreadyExecuted = false;
        this.mTroughCount     = 0;
        this.mPeakCount       = 0;
        this.mPeakHold        = 0;
        this.mTroughHold      = 0;
        this.mCalcDiff        = 0;
        this.mCountHold       = 0;
        this.mPeakFrame       = 0;
        this.mTroughFrame     = 0;
        this.mSignalWidth     = 0;
        this.mPeak            = 0;
        this.mTrough          = 0;
        this.mFrameCount      = mFrameCount;
        this.mConfidence      = new int[MAX_ERROR_ALLOTMENT];
        this.mElementHolder   = new LinkedList();
        this.mFilteredData    = new ArrayList<>();
        this.mOriginalData    = new ArrayList<>();
    }


    public void updateFrameCount(int frameCount) {
        if (mFrameCount < 0) {
            throw new IllegalArgumentException();
        }
        this.mFrameCount = frameCount;
    }

    /**
     * Determines the peaks and troughs of the incoming
     * raw signal. Ignores the first "n" frames due to
     * excessive noise.
     *
     * @param rAvg The average red value (0 - 255 RGB format)
     *             of each input video frame
     *
     * @return An array that contains three data points:
     *          double[0] contains either a 0.0, 1.0 or 2.0
     *                    0.0 -  signifies an arbitrary data point
     *                    1.0 -  signifies a peak point
     *                    2.0 -  signifies a trough point
     *          double[1] contains the frame count
     *          double[2] contains the r value
     *
     * An array with all indexes set to 0 signifies a point
     * that is NOT a peak NOR trough.
     */
    public double[] detectPeakTrough(double rAvg) {
        double[] result = new double[3];
        if (mFrameCount <= GARBAGE_FRAMES) {
            return result;
        }
        if (GARBAGE_FRAMES + 1 == mFrameCount) {
            mLpfOutput = rAvg;
        } else {
            mLpfOutput = lowPassFilter(LOW_PASS_SMOOTHING, rAvg, mLpfOutput);
            if (GARBAGE_FRAMES + 2 == mFrameCount) {
                mHpfOutput = 0;
                mPrevLpfInput = mLpfOutput;
            } else {
                mHpfOutput = highPassFilter(HIGH_PASS_SMOOTHING, mLpfOutput,
                                            mHpfOutput, mPrevLpfInput);
                mPrevLpfInput = mLpfOutput;
                if (mFrameCount <= GARBAGE_FRAMES + DETECTOR_BUFFER_SIZE + 2) {
                    mFilteredData.add(mHpfOutput);
                    mOriginalData.add(rAvg);
                } else {
                    final int rMapping = mFrameCount - MAPPING;
                    final double rVal = mOriginalData.get(2);
                    if (mFilteredData.get(2) > mFilteredData.get(1) &&
                        mFilteredData.get(2) > mFilteredData.get(3) &&
                        mFilteredData.get(1) > mFilteredData.get(0) &&
                        mFilteredData.get(3) > mFilteredData.get(4)) {
                        result[0] = PEAK;
                        result[1]= rMapping;
                        result[2] = rVal;
                    } else if (mFilteredData.get(2) < mFilteredData.get(1) &&
                               mFilteredData.get(2) < mFilteredData.get(3) &&
                               mFilteredData.get(1) < mFilteredData.get(0) &&
                               mFilteredData.get(3) < mFilteredData.get(4)) {
                        result[0] = TROUGH;
                        result[1]= rMapping;
                        result[2] = rVal;
                    }
                    mOriginalData.remove(0);
                    mFilteredData.remove(0);
                    mOriginalData.add(rAvg);
                    mFilteredData.add(mHpfOutput);
                }
            }
        }
        return result;
    }


    /**
     * Detects whether the user has moved his/her finger
     * from the camera. Ignores the first "n" frames
     * due to excessive noise. Will return an integer
     * value that has:
     *
     * 1 - Finger is accurately on camera!
     * 2 - Finger not covering camera correctly!
     * 3 - Finger has shifted on camera!
     * 4 - Finger not on camera!
     *
     * 0 - No result
     *
     * @param rVal Average red value of a single image (0 - 255 RGB format)
     * @param gVal Average green value of a single image (0 - 255 RGB format)
     * @param bVal Average blue value of a single image (0 - 255 RGB format)
     * @param confidence Accuracy level of system (1 = 10% ... 9 = 90%)
     */

    public int checkDataQuality(double rVal, double gVal, double bVal, int confidence) {
        int output = 0;
        if (rVal < 0 || gVal < 0 || bVal < 0 || confidence > DATA_SIZE - 1 || confidence < 0) {
            throw new IllegalArgumentException();
        }
        if (mPeakCount < AVERAGE || mTroughCount < AVERAGE) {
            double[] result = detectPeakTrough(rVal);
            if (result[0] == PEAK) {
                mPeakHold += result[2];
                mPeakFrame += result[1];
                mPeakCount++;
            } else if(result[0] == TROUGH) {
                mTroughHold += result[2];
                mTroughFrame += result[1];
                mTroughCount++;
            }
        } else if (!mAlreadyExecuted) {
            mCalcDiff = ((mPeakHold / mPeakCount) - (mTroughHold / mTroughCount)) + ERROR_TOLERANCE;
            mSignalWidth = (int) Math.abs((((mPeakFrame / mPeakCount)
                                 - (mTroughFrame / mTroughCount))) + ERROR_TOLERANCE);
            mCountHold = mFrameCount;
            mAlreadyExecuted = true;
        } else {
            if (mFrameCount - mCountHold <= mSignalWidth) {
                mElementHolder.add(rVal);
            } else {
                double difference = Math.abs(rVal - mElementHolder.remove());
                if (difference < mCalcDiff && difference > MINIMUM_DIFFERENCE &&
                        gVal < MAX_LIGHT_COVER && bVal < MAX_LIGHT_COVER) {
                    mConfidence[0]++;
                } else if (difference < mCalcDiff && difference > MINIMUM_DIFFERENCE &&
                        (gVal < MAX_LIGHT_PARTIAL_COVER || bVal < MAX_LIGHT_PARTIAL_COVER)) {
                    mConfidence[1]++;
                } else if (difference > mCalcDiff) {
                    mConfidence[2]++;
                } else { 
                    mConfidence[3]++;
                }
                mElementHolder.add(rVal);
                if (mFrameCount % DATA_SIZE == 0) {
                   output = confidenceCheck(confidence);
                    mConfidence = new int[MAX_ERROR_ALLOTMENT];
                }
            }
        }
        return output;
    }

    // Error reporting
    private int confidenceCheck(int confidence) {
        int result = 0;
        if (mConfidence[0] >= confidence) {
            result = 1;
        } else if (mConfidence[1] >= confidence) {
            result = 2;
        } else if (mConfidence[2] >= confidence) {
            result = 3;
        } else if (mConfidence[3] >= confidence) {
            result = 4;
        }
        return result;
    }

    /**
     * *** THEORY SHOWN BELOW ***
     *
     * Low pass filter that removes high
     * frequency noise. Based on the following
     * difference equation below:
     *
     * y[n] = y[n - 1] + (x[n] - y[n - 1]) / a
     *
     * 'y[n]' represents the current output signal
     * 'x[n]' represents the current input signal
     * 'a'    a constant that determines the
     *        sharpness of the output peaks/troughs
     *
     * @param smoothing  filter aggressiveness
     * @param input      noisy signal to be filtered
     * @param prevOutput previous value of the clean signal
     * @return filtered signal
     */
    private double lowPassFilter(double smoothing, double input, double prevOutput) {
        if (smoothing <= 1) {
            throw new IllegalArgumentException();
        }
        return prevOutput + (input - prevOutput) / smoothing;
    }

    /**
     * *** THEORY SHOWN BELOW ****
     *
     * High pass filter that removes low
     * frequency noise. Based on the following
     * difference equation below:
     *
     * y[n] = b * (y[n - 1] + x[n] - x[n - 1])
     *
     * 'y[n]' represents the current output signal
     * 'x[n]' represents the current input signal
     * 'b'    equivalent to (1 - (1 / smoothing));
     *        determines sharpness of output peaks/troughs
     *
     * @param smoothing  filter aggressiveness
     * @param input      noisy signal to be filtered
     * @param prevInput  previous value of the noisy signal
     * @param prevOutput previous value of the clean signal
     * @return filtered signal
     */
    private double highPassFilter(double smoothing, double input,
                                  double prevOutput, double prevInput) {
        if (smoothing <= 1) {
            throw new IllegalArgumentException();
        }
        double b = 1 - (1 / smoothing);
        return b * (prevOutput + input - prevInput);
    }

    /**
     * Outputs a difference (magnitude) of the AC component of
     * the input signal. The difference is calculated by subtracting
     * the peak from the trough.
     *
     * @param input the array that contains
     * @return the difference between the peak and trough
     */
    public double findAcRange(double[] input) {
        double difference = 0.0;
        if (input[0] == PEAK && !mPrevPeakDetected) {
            mPeak = input[2];
            mPrevPeakDetected = true;
        } else if (input[0] == TROUGH && !mPrevTroughDetected) {
            mTrough = input[2];
            mPrevTroughDetected = true;
        }
        if (mPrevTroughDetected && mPrevPeakDetected) {
            difference = Math.abs(mPeak - mTrough);
            mPrevPeakDetected = false;
            mPrevTroughDetected = false;
        }
        return difference;
    }
}

