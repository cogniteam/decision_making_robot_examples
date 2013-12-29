/*
 * Filename: WanderingNode.cpp
 *   Author: Igor Makhtes
 *     Date: Dec 10, 2013
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Cogniteam Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <iostream>
#include <time.h>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time.hpp>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include "Wandering.h"

using namespace std;

#define foreach BOOST_FOREACH

ros::Subscriber laserSub;
ros::Publisher  velocityPub;
ros::NodeHandle* node;

void publishVelocity(double linear, double angular) {
    geometry_msgs::Twist velocity;
    velocity.linear.x = linear;
    velocity.angular.z = angular;

    for (int i = 0; i < 10; i++)
        velocityPub.publish(velocity);
}

/**
 * Preemptive wait
 * @param ms Milliseconds to wait
 * @param queue EventQueue
 * @return True if preempted
 */
bool preemptiveWait(double ms, decision_making::EventQueue& queue) {
    for (int i = 0; i < 100 && !queue.isTerminated(); i++)
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 100.0));

    return queue.isTerminated();
}

/**
 * Preemptive wait
 * @param ms Milliseconds to wait
 * @param queue EventQueue
 * @return True if preempted
 */
bool preemptiveWait(double minMs, double maxMs, decision_making::EventQueue& queue) {
    double msToWait = ((double)rand() / (double)RAND_MAX) * (maxMs - minMs) + minMs;
    return preemptiveWait(msToWait, queue);
}

double getAverage(int centerRange, int rangeSize, const sensor_msgs::LaserScan::Ptr& scan) {
    double sum = std::accumulate(scan->ranges.begin() + centerRange - rangeSize / 2.0,
                                 scan->ranges.begin() + centerRange + rangeSize / 2.0, 0.0);

    return sum / (double)rangeSize;
}

int countRanges(const sensor_msgs::LaserScan::Ptr& scan, int center, int range, double lessThan) {
    return std::count_if(scan->ranges.begin() + center - range / 2.0,
                         scan->ranges.begin() + center + range / 2.0,
                         bind2nd(less<double>(), lessThan));
}

void onLaserScan(const sensor_msgs::LaserScan::Ptr scan, EventQueue* q) {
    int countFrontRange = countRanges(scan, scan->ranges.size() / 2, 100, 0.3);
    int countRightRange = countRanges(scan, (scan->ranges.size() / 6) * 1, 50, 0.3);
    int countLeftRange  = countRanges(scan, (scan->ranges.size() / 6) * 5, 50, 0.3);

    bool front = false;
    bool left = false;
    bool right = false;

    if (countFrontRange > 5) {
        front = true;
        q->riseEvent("/FRONT_OBSTACLE");
    }

    if (countLeftRange > 3) {
        left = true;
        q->riseEvent("/LEFT_OBSTACLE");
    }

    if (countRightRange > 3) {
        right = true;
        q->riseEvent("/RIGHT_OBSTACLE");
    }

    if (front && right)
        q->riseEvent("/FRONT_AND_RIGHT_OBSTACLE");

    if (front && left)
        q->riseEvent("/FRONT_AND_LEFT_OBSTACLE");
}

decision_making::TaskResult stopRobot(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(0, 0);
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult turnRight(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(0, -2);

    if (preemptiveWait(500, 1000, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_TURN");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult turnLeft(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(0, 2);

    if (preemptiveWait(500, 1000, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_TURN");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult turnRandom(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    bool turnRight = rand() % 2;
    publishVelocity(0, (turnRight * -2 + 1) * 2);

    ROS_INFO("Turning randomly");

    if (preemptiveWait(500, 2500, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_TURN");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult drive(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(0.75, 0);

    if (preemptiveWait(9000, 20000, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_DRIVE");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult driveBackward(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(-0.75, 0);

    if (preemptiveWait(1000, 2000, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_BACKWARD");
    return decision_making::TaskResult::SUCCESS();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wandering_node");
    ros_decision_making_init(argc, argv);

    boost::posix_time::ptime epoch(boost::posix_time::min_date_time);
    boost::posix_time::ptime now(boost::posix_time::microsec_clock::local_time());
    unsigned int seed = (now - epoch).total_nanoseconds();
    srand(seed);
    node = new ros::NodeHandle();

    RosEventQueue* q = new RosEventQueue();
    laserSub    = node->subscribe<void>("/scan", 10,
            boost::function<void(const sensor_msgs::LaserScan::Ptr)>(boost::bind(onLaserScan, _1, q)));

    velocityPub = node->advertise<geometry_msgs::Twist>("/cmd_vel", 10, false);

    LocalTasks::registrate("TurnRight", turnRight);
    LocalTasks::registrate("TurnLeft", turnLeft);
    LocalTasks::registrate("TurnRandom", turnRandom);
    LocalTasks::registrate("Drive", drive);
    LocalTasks::registrate("DriveBackward", driveBackward);
    LocalTasks::registrate("StopRobot", stopRobot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    FsmWandering(NULL, q, "Wandering");

    delete node;
    return 0;
}
