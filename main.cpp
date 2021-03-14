#include <iostream>
#include <ctime>
#include <fstream>
#include <unistd.h>
#include <vector>
#include <math.h>

#include "wiringPi.h"
#include "raspicam/raspicam_cv.h"



using namespace std;

//sets basic settings on camera
void CameraSetup(raspicam::RaspiCam_Cv &camera)
{
    camera.set(cv::CAP_PROP_FORMAT, CV_8UC1);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 800);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 640);
}

//goes trough collected values, finds those that are valid, and writes their position in collumn to a map vector
vector<vector<int>> CreateCorrectZValueMap(const vector<vector<int>> &high_values_position_list)
{
    vector<vector<int>> vector_map;
    for (int vector_idx = 0; vector_idx < int(high_values_position_list.size()); vector_idx++) {
        vector<int> value_map;
        value_map.empty();
        for(int value_idx = 0; value_idx < high_values_position_list[vector_idx].size(); value_idx++) {
            if(high_values_position_list[vector_idx][value_idx] > 0){//picks out correct values and adds them to value map
                value_map.push_back(value_idx);
            }
        }
        vector_map.push_back(value_map);
    }
    return vector_map;
}

//vector<float> => x;y;z //vector<vector<float>> => vortex column //vector<vector<vector<float>>> all vertexes proper explanation for maths and logic in documentation
vector<vector<vector<float>>> CreateVertexVector(const vector<vector<int>> &position_vector, const vector<vector<int>> &height_vector, const float &laser_angle, const float &degree_change)
{
    vector<vector<vector<float>>> vertex_vector;
    cout<<height_vector.size()<<endl;
    cout<<height_vector[1].size()<<endl;
    sleep(2);

    for (int movement = 0;movement < height_vector.size(); movement++) {
        vector<vector<float>> vertex_column;
        vertex_column.empty();
        float conversion_to_rad = 3.14159265 / 180;
        for (int position = 0; position < height_vector[movement].size(); position++) {
            int height_value = height_vector[movement][position];
            float z = (440 - height_value) * 3;
            int temp = position_vector[movement][height_value];
            temp = (abs(400-temp));
            //cout<< temp << endl;
            float r = float(temp)/float(cos(float(laser_angle) * conversion_to_rad));
            float curr_angle = float(movement) * degree_change;
            float temp_angle = ((float(180) - curr_angle)/2);
            float long_side = 2*r*cos(temp_angle * conversion_to_rad);
            float position_angle = float(90) - temp_angle;
            float y = long_side * sin(position_angle * conversion_to_rad);
            float x = long_side * cos(position_angle * conversion_to_rad);
            vector<float> vertex;
            vertex.empty();
            vertex.push_back(x);
            vertex.push_back(y);
            vertex.push_back(z);
            vertex_column.push_back(vertex);
        }
        vertex_vector.push_back(vertex_column);

    }
    return vertex_vector;
}
void WriteTriangle(ofstream &output_file,const vector<float> &first_vertex, const vector<float> &second_vertex, const vector<float> &third_vertex){
    output_file << "facet normal 0 0 0" << endl;
    output_file << "outer loop" << endl;
    output_file << "vertex " <<  first_vertex[0] << " " << first_vertex[1] << " " << first_vertex[2] << endl;
    output_file << "vertex " <<  second_vertex[0] << " " << second_vertex[1] << " " << second_vertex[2] << endl;
    output_file << "vertex " <<  third_vertex[0] << " " << third_vertex[1] << " " << third_vertex[2] << endl;
    output_file << "endloop" << endl;
    output_file << "endfacet" << endl;
}
int main()
{
    raspicam::RaspiCam_Cv camera;
    vector<vector<int>> high_values_position_list;
    wiringPiSetupGpio();
    pinMode(13, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);

    pwmSetRange(2000);
    pwmSetClock(192);

    cv::Mat image;
    //50 LEFT; 250 RIGHT
    //uchar visible_change[] = "255";//value used to check values on picture during debug
    const float degree_change = 0.8955224f;
    const float laser_angle = 75;
    int pwm_value = 50;
    CameraSetup(camera);

    for (int reset = 1; reset < 9; reset++) {
        pwmWrite(13, 250-(25*reset));
        sleep(1);
    }
    for (int wait = 0; wait < 10; wait++) {
        cout << ". ";
        sleep(1);
        cout << endl;

    }
    while(pwm_value <= 250) {
        vector<int> high_values_position;
        high_values_position.empty();
        pwmWrite(13, pwm_value);

        cout << "servo moved to " << pwm_value << endl;
        high_values_position.empty();
        camera.open();

        sleep(1);//gives time to camera to gather light
        if(camera.isOpened()) {
            camera.grab();
        } else {
        cout << "camera is closed" << endl;
        }
        camera.retrieve(image);//retrieves image from camera

        camera.release();
        const int n_of_rows = 550;                                  //*
        const int n_of_cols = 400;                                  //*
        for (int row_idx = 200; row_idx < n_of_rows; row_idx++) {   //*
            int top_value = 0;
            int top_value_position = 0;
            for (int col_idx = 50; col_idx < n_of_cols; col_idx++) {//*=> limits the place, where we search for values 
                int temp = int(image.at<uchar>(row_idx, col_idx));
                if(temp > top_value) {// we search for highest value in row
                    top_value = temp;
                    top_value_position = col_idx;
                }
            }
            cout << "color value: "<< top_value << endl;
            if(top_value < 50){// we set values that are too weak to 0(which we intepret as useless values later on)
                top_value_position = 0;
            }
            cout << "position value: "<< top_value_position << endl;
            high_values_position.push_back(top_value_position);// we save the pixel position in a row
        }

        high_values_position_list.push_back(high_values_position);// once we finish processing the picture, we save collected data and move on next one
        pwm_value++;
    }

    vector<vector<int>> correct_values_map = CreateCorrectZValueMap(high_values_position_list);
    vector<vector<vector<float>>> vertex_map = CreateVertexVector(high_values_position_list, correct_values_map, laser_angle, degree_change);
    vector<vector<float>> top_row;
    vector<vector<float>> bottom_row;
    top_row.empty();
    bottom_row.empty();
    for (int vector_idx = 0; vector_idx < vertex_map.size(); vector_idx++) {
        top_row.push_back(vertex_map[vector_idx][0]);
        bottom_row.push_back(vertex_map[vector_idx][vertex_map[vector_idx].size() - 1]);
    }
    ofstream output_file("out.stl");
    output_file << "solid scan" << endl;

    for (int vertex_vector_idx = 0; vertex_vector_idx < vertex_map.size(); ++vertex_vector_idx) {
        if(vertex_vector_idx != vertex_map.size()-1) {
            if(vertex_map[vertex_vector_idx].size() < vertex_map[vertex_vector_idx + 1].size()) {
                    bool two_on_left = false;
                    vector<float> vertex_left_side = vertex_map[vertex_vector_idx][0];
                    vector<float> vertex_right_side = vertex_map[vertex_vector_idx + 1][0];
                    vector<float> vertex_variable  = vertex_map[vertex_vector_idx + 1][1];
                    int temp = 1;
                    WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);


                    bool do_once = true;
                    while(temp < vertex_map[vertex_vector_idx + 1].size()) {
                        if(temp < vertex_map[vertex_vector_idx].size()) {
                            if(two_on_left) {
                                vertex_left_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx + 1][temp];
                                two_on_left = false;
                            } else {
                                vertex_right_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx][temp];
                                two_on_left = true;
                                temp++;
                            }
                        } else {
                            if(do_once){
                                vertex_left_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx + 1][temp];
                                do_once = false;
                            } else {
                                vertex_right_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx + 1][temp];
                            }
                            temp++;

                        }
                        WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);

                    }
            } else if(vertex_map[vertex_vector_idx].size() > vertex_map[vertex_vector_idx + 1].size()) {
                    bool two_on_left = true;
                    vector<float> vertex_left_side = vertex_map[vertex_vector_idx][0];
                    vector<float> vertex_right_side = vertex_map[vertex_vector_idx + 1][0];
                    vector<float> vertex_variable  = vertex_map[vertex_vector_idx][1];
                    int temp = 1;
                    WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);

                    bool do_once = true;
                    while(temp < vertex_map[vertex_vector_idx].size()) {
                        if(temp < vertex_map[vertex_vector_idx + 1].size()) {
                            if(two_on_left) {
                                vertex_left_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx + 1][temp];
                                two_on_left = false;
                                temp++;
                            } else {
                                vertex_right_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx][temp];
                                two_on_left = true;         }
                        } else {
                            if(do_once){
                                vertex_right_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx][temp];
                                do_once = false;
                            } else {
                                vertex_left_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx][temp];
                            }
                            temp++;

                        }
                        WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);
                    }
            } else {
                    bool two_on_left = true;
                    vector<float> vertex_left_side = vertex_map[vertex_vector_idx][0];
                    vector<float> vertex_right_side = vertex_map[vertex_vector_idx + 1][0];
                    vector<float> vertex_variable  = vertex_map[vertex_vector_idx][1];
                    int temp = 1;
                    WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);
                    bool do_once = true;
                    while(temp < vertex_map[vertex_vector_idx].size()) {
                        if(two_on_left) {
                            vertex_left_side = vertex_variable;
                            vertex_variable = vertex_map[vertex_vector_idx + 1][temp];
                            two_on_left = false;
                            temp++;
                        } else {
                            vertex_right_side = vertex_variable;
                            vertex_variable = vertex_map[vertex_vector_idx][temp];
                            two_on_left = true;
                        }
                        WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);
                    }
                }
        } else {
                if(vertex_map[vertex_vector_idx].size() < vertex_map[0].size()) {
                    bool two_on_left = false;
                    vector<float> vertex_left_side = vertex_map[vertex_vector_idx][0];
                    vector<float> vertex_right_side = vertex_map[0][0];
                    vector<float> vertex_variable  = vertex_map[0][1];
                    int temp = 1;
                    WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);
                    bool do_once = true;
                    while(temp < vertex_map[0].size()) {
                        if(temp < vertex_map[vertex_vector_idx].size()) {
                            if(two_on_left) {
                                vertex_left_side = vertex_variable;
                                vertex_variable = vertex_map[0][temp];
                                two_on_left = false;
                            } else {
                                vertex_right_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx][temp];
                                two_on_left = true;
                                temp++;
                                                            }
                        } else {
                            if(do_once){
                                vertex_left_side = vertex_variable;
                                vertex_variable = vertex_map[0][temp];
                                do_once = false;
                            } else {
                                vertex_right_side = vertex_variable;
                                vertex_variable = vertex_map[0][temp];
                            }
                            temp++;

                        }
                        WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);

                    }
                } else if(vertex_map[vertex_vector_idx].size() > vertex_map[0].size()) {
                    bool two_on_left = true;
                    vector<float> vertex_left_side = vertex_map[vertex_vector_idx][0];
                    vector<float> vertex_right_side = vertex_map[0][0];
                    vector<float> vertex_variable  = vertex_map[vertex_vector_idx][1];
                    int temp = 1;
                    WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);
                    bool do_once = true;
                    while(temp < vertex_map[vertex_vector_idx].size()) {
                        if(temp < vertex_map[0].size()) {
                            if(two_on_left) {
                                vertex_left_side = vertex_variable;
                                vertex_variable = vertex_map[0][temp];
                                two_on_left = false;
                                temp++;
                            } else {
                                vertex_right_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx][temp];
                                two_on_left = true;         }
                        } else {
                            if(do_once){
                                vertex_right_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx][temp];
                                do_once = false;
                            } else {
                                vertex_left_side = vertex_variable;
                                vertex_variable = vertex_map[vertex_vector_idx][temp];
                            }
                            temp++;

                        }
                        WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);

                    }
                } else {
                    bool two_on_left = true;
                    vector<float> vertex_left_side = vertex_map[vertex_vector_idx][0];
                    vector<float> vertex_right_side = vertex_map[0][0];
                    vector<float> vertex_variable  = vertex_map[vertex_vector_idx][1];
                    int temp = 1;
                    WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);

                    bool do_once = true;
                    while(temp < vertex_map[vertex_vector_idx].size()) {
                        if(two_on_left) {
                            vertex_left_side = vertex_variable;
                            vertex_variable = vertex_map[0][temp];
                            two_on_left = false;
                            temp++;
                        } else {
                            vertex_right_side = vertex_variable;
                            vertex_variable = vertex_map[vertex_vector_idx][temp];
                            two_on_left = true;
                        }
                        WriteTriangle(output_file, vertex_left_side, vertex_variable, vertex_right_side);
                    }
                }
            }
    }

    for(int top_idx = 1; top_idx < top_row.size() - 2; top_idx++) {
        vector<float> base_top_vertex = top_row[0];
        WriteTriangle(output_file, base_top_vertex, top_row[top_idx], top_row[top_idx + 1]);


    }
    for(int bottom_idx = 1; bottom_idx < top_row.size() - 2; bottom_idx++) {
        vector<float> base_bottom_vertex = bottom_row[0];
        WriteTriangle(output_file, base_bottom_vertex, bottom_row[bottom_idx], bottom_row[bottom_idx + 1]);
    }

    output_file << "endsolid scan";
    return 0;
}
