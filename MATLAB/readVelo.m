function [ output_args ] = readVelo( input_args )
%READVELO Summary of this function goes here
%   Detailed explanation goes here

velo = csvread('/home/sebastian/Git/rosbuild_ws/loam/loam_velodyne/input_0.csv.csv');
end

