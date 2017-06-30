/*
Copyright (c) 2014, Nghia Ho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <sys/time.h>
#include <pthread.h>
#include <math.h>
#include <cstdlib>
#define GB  1024l * 1024l  * 1024  // 1GB

using namespace std;
using namespace cv;

// This video stablisation smooths the global trajectory using a sliding average window

const int SMOOTHING_RADIUS = 30; // In frames. The larger the more stable the video, but less reactive to sudden panning
const int HORIZONTAL_BORDER_CROP = 20; // In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.

// 1. Get previous to current frame transformation (dx, dy, da) for all frames
// 2. Accumulate the transformations to get the image trajectory
// 3. Smooth out the trajectory using an averaging window
// 4. Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
// 5. Apply the new transformation to the video

struct TransformParam
{
    TransformParam() {}
    TransformParam(double _dx, double _dy, double _da) {
        dx = _dx;
        dy = _dy;
        da = _da;
    }

    double dx;
    double dy;
    double da; // angle
};

struct Trajectory
{
    Trajectory() {}
    Trajectory(double _x, double _y, double _a) {
        x = _x;
        y = _y;
        a = _a;
    }

    double x;
    double y;
    double a; // angle
};


struct Data
{
    Data() {}
    long rank;
    vector <Mat> *frames;
    vector <TransformParam> *prev_to_cur_transform;
    int nt;

};

double rtclock()
{
    struct timezone Tzp;
    struct timeval Tp;
    int stat;
    stat = gettimeofday (&Tp, &Tzp);
    if (stat != 0) printf("Error return from gettimeofday: %d",stat);
    return(Tp.tv_sec + Tp.tv_usec*1.0e-6);
}


/*Carrega um limite de quadros para memoria, dado por memory_limit*/
vector <Mat> readFrames(VideoCapture cap, long memory_limit)
{
    vector <Mat> frames;
    long count_memory = 0l;
    while(count_memory < memory_limit) {
        Mat cur;
        cap >> cur;
        if(cur.data == NULL) {
            break;
        }
        frames.push_back(cur);
        count_memory  +=  cur.total() * cur.elemSize(); // EM Bytes
    }
    return frames;
}

//Calcula transformaca nos frames
void * transformationFrame(void * data )
{
    Mat  cur_grey, prev_grey;
    struct Data *d = (struct Data*)data;

    long thread_id = d->rank;
    long offset = d->frames->size() / d->nt;
    long begin =  offset * thread_id;
    long end;

    if (thread_id + 1 == d->nt){
        end = begin + offset + d->frames->size() % d->nt;
    } else{
        end = begin + offset;
    }

    Mat last_T =  (Mat_<double>(2,3) << 1, 0, 0, 0, 1, 0);
    for (int j = begin+1; j < end; j++){
        cvtColor( ( * d->frames)[j-1], prev_grey,   COLOR_BGR2GRAY);
        cvtColor( ( * d->frames)[j],   cur_grey,   COLOR_BGR2GRAY);
        // vector from prev to cur
        vector <Point2f> prev_corner, cur_corner;
        vector <Point2f> prev_corner2, cur_corner2;
        vector <uchar> status;
        vector <float> err;

        goodFeaturesToTrack (prev_grey, prev_corner, 200, 0.01, 30);
        calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

        // weed out bad matches
        for(size_t i=0; i < status.size(); i++) {
            if(status[i]) {
                prev_corner2.push_back(prev_corner[i]);
                cur_corner2.push_back(cur_corner[i]);
            }
        }

        // translation + rotation only (homografia)
        Mat T = estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

        // in rare cases no transform is found. We'll just use the last known good transform.
        if(T.data == NULL) {
            last_T.copyTo(T);

        } else {
            T.copyTo(last_T);
        }

        // decompose T
        double dx = T.at<double>(0,2);
        double dy = T.at<double>(1,2);
        double da = atan2(T.at<double>(1,0), T.at<double>(0,0));
        ( * d->prev_to_cur_transform)[j-1] = (TransformParam(dx, dy, da));

    }
}

/* Calculates the trajectory */
vector <Trajectory> accumulateTransformations(
                    vector <TransformParam> prev_to_cur_transform)
{
    // Accumulated frame to frame transform
    double a = 0;
    double x = 0;
    double y = 0;
    Mat  cur;

    vector <Trajectory> trajectory; // trajectory at all frames

    for(size_t i=0; i < prev_to_cur_transform.size(); i++) {
        x += prev_to_cur_transform[i].dx;
        y += prev_to_cur_transform[i].dy;
        a += prev_to_cur_transform[i].da;

        trajectory.push_back(Trajectory(x,y,a));
    }
    return trajectory;
}

/* Calculates smoothed trajectory */
vector <Trajectory> smoothedTrajectory
(
                    vector <Trajectory> trajectory)
{
    vector <Trajectory> smoothed_trajectory; // trajectory at all frames

    for(size_t i=0; i < trajectory.size(); i++) {
        double sum_x = 0;
        double sum_y = 0;
        double sum_a = 0;
        int count = 0;

        for(int j=-SMOOTHING_RADIUS; j <= SMOOTHING_RADIUS; j++) {
            if(i+j >= 0 && i+j < trajectory.size()) {
                sum_x += trajectory[i+j].x;
                sum_y += trajectory[i+j].y;
                sum_a += trajectory[i+j].a;

                count++;
            }
        }

        double avg_a = sum_a / count;
        double avg_x = sum_x / count;
        double avg_y = sum_y / count;
        smoothed_trajectory.push_back(Trajectory(avg_x, avg_y, avg_a));
    }

    return smoothed_trajectory;
}

/* Calculates a new transformation */
vector <TransformParam> newTransformation(
                    vector <TransformParam> prev_to_cur_transform,
                    vector <Trajectory> smoothed_trajectory
                    )
{
    double a = 0;
    double x = 0;
    double y = 0;
    vector <TransformParam> new_prev_to_cur_transform;

    for(size_t i=0; i < prev_to_cur_transform.size(); i++) {
        x += prev_to_cur_transform[i].dx;
        y += prev_to_cur_transform[i].dy;
        a += prev_to_cur_transform[i].da;

        // target - current
        double diff_x = smoothed_trajectory[i].x - x;
        double diff_y = smoothed_trajectory[i].y - y;
        double diff_a = smoothed_trajectory[i].a - a;

        double dx = prev_to_cur_transform[i].dx + diff_x;
        double dy = prev_to_cur_transform[i].dy + diff_y;
        double da = prev_to_cur_transform[i].da + diff_a;

        new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
    }

    return new_prev_to_cur_transform;

}

/* Applies the new transformation in the video */
void applyTransformation(VideoCapture cap,
                         vector <TransformParam> new_prev_to_cur_transform,
                         int vert_border,
                         int count_frames
                        )
{
    cap.set(CV_CAP_PROP_POS_FRAMES, 0);
    Mat T(2,3,CV_64F);

    int k = 0;
    Mat cur;

    while(k < count_frames-1) { // don't process the very last frame, no valid transform
        cap >> cur;

        if(cur.data == NULL) {
            break;
        }

        T.at<double>(0,0) = cos(new_prev_to_cur_transform[k].da);
        T.at<double>(0,1) = -sin(new_prev_to_cur_transform[k].da);
        T.at<double>(1,0) = sin(new_prev_to_cur_transform[k].da);
        T.at<double>(1,1) = cos(new_prev_to_cur_transform[k].da);

        T.at<double>(0,2) = new_prev_to_cur_transform[k].dx;
        T.at<double>(1,2) = new_prev_to_cur_transform[k].dy;

        Mat cur2;

        warpAffine(cur, cur2, T, cur.size());

        cur2 = cur2(Range(vert_border, cur2.rows-vert_border), Range(HORIZONTAL_BORDER_CROP, cur2.cols-HORIZONTAL_BORDER_CROP));

        // Resize cur2 back to cur size, for better side by side comparison
        resize(cur2, cur2, cur.size());

        // Now draw the original and stablised side by side for coolness
        Mat canvas = Mat::zeros(cur.rows, cur.cols*2+10, cur.type());

        cur.copyTo(canvas(Range::all(), Range(0, cur2.cols)));
        cur2.copyTo(canvas(Range::all(), Range(cur2.cols+10, cur2.cols*2+10)));

        // If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
        if(canvas.cols > 1920) {
            resize(canvas, canvas, Size(canvas.cols/2, canvas.rows/2));
        }

        imshow("before and after", canvas);
        waitKey(20);
        k++;
    }

}

int main(int argc, char **argv)
{
    /*local variables*/
    double t_run =0.0,  t_r = 0.0;
    pthread_t* threads ;
    vector <Mat> frames;
    Mat last_frame;
    long count_frames = 0;
    long nt=0;
    long memory_limit;

    /*Put opecnv in sequential, non-optimized mode*/
    setNumThreads(0);
    setUseOptimized (0);

    /*Checks if file name passed by command line*/
    if(argc < 4) {
        cout << "./VideoStab [video.avi]" << endl;
        cout << "Memory Limit" << endl;
        cout << "Number of threads" << endl;

        return 0;
    }

    VideoCapture cap(argv[1]);
    memory_limit = atol (argv[2]);
    nt = atol (argv[3]);
    threads = new pthread_t [nt];
    assert(cap.isOpened());

    frames = readFrames(cap, memory_limit * GB);

    vector <TransformParam> prev_to_cur_transform;
    vector <TransformParam> *prev_to_cur_transform_tmp = new vector<TransformParam>(frames.size()-1);

    /* Allocates data structure for threads arguments */
    vector <Data *> datas;
    for(long i=0; i < nt; i++ ){
        Data *data = new Data;
        datas.push_back(data);
    }

    while( frames.size() > 0 ){
        count_frames += frames.size()-1;
        t_r = rtclock();
        for(long i=0; i < nt; i++ ){
            datas[i]->nt = nt;
            datas[i]->rank = i;
            datas[i]->frames = &frames;
            datas[i]->prev_to_cur_transform = prev_to_cur_transform_tmp;
            pthread_create(&threads[i], NULL, transformationFrame, (void *)datas[i]);
        }
        for(int i=0; i < nt; i++ ){
            pthread_join(threads[i], NULL);
        }
        t_run += rtclock() - t_r;
        prev_to_cur_transform.insert(prev_to_cur_transform.end(),
                                     prev_to_cur_transform_tmp->begin(),
                                     prev_to_cur_transform_tmp->end());

        /*Take the last frame of this slice */
        last_frame = frames[frames.size()-1];

        /*Free memory*/
        frames.clear();

        /*Read the missing frames*/
        frames = readFrames(cap, memory_limit * GB);

        if (frames.size() > 0){
            /* Copy the last frame to the first position of the new slice */
            frames.insert(frames.begin(), last_frame);
            prev_to_cur_transform_tmp->resize(frames.size()-1);

        }
    }

    cout << std::setprecision(6) << t_run << endl;

    // Step 2 - Accumulate the transformations to get the image trajectory
    // Accumulated frame to frame transform
    vector <Trajectory> trajectory = accumulateTransformations(prev_to_cur_transform); // trajectory at all frames

    // Step 3 - Smooth out the trajectory using an averaging window
    vector <Trajectory> smoothed_trajectory = smoothedTrajectory(trajectory); // trajectory at all frames

    // Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
    vector <TransformParam> new_prev_to_cur_transform = newTransformation(prev_to_cur_transform, smoothed_trajectory);

    int vert_border = HORIZONTAL_BORDER_CROP * last_frame.rows / last_frame.cols; // get the aspect ratio correct
    // Step 5 - Apply the new transformation to the video
    applyTransformation(cap, new_prev_to_cur_transform, vert_border, count_frames);

    delete [] threads;
    return 0;
}
