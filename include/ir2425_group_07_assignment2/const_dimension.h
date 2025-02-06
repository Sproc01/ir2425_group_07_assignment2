// Author: Michele Sprocatti
#ifndef CONST_DIMENSION_H
#define CONST_DIMENSION_H

// Torso joint home value and offset value
const double TORSO_HOME_VALUE = 0.33;
const double TORSO_OFFSET_VALUE = 0.06;

// Dimensions of the objects
const double HEXAGONAL_RADIUS = 0.05; //0.025;
const double HEXAGONAL_HEIGHT = 0.1;

const double CUBE_SIDE = 0.05;

const double TRIANGULAR_HEIGHT = 0.035;
const double TRIANGULAR_BASE = 0.07;
const double TRIANGULAR_SIDE = 0.05;


// Dimensions and positions of the tables
const double TABLE_SIDE = 0.9;

const double TABLE_PICK_X = 7.85;
const double TABLE_PICK_Y = -3;

const double TABLE_PLACE_X = 7.85;
const double TABLE_PLACE_Y = -1.9;

const double TABLE_Z = 0.35;

// name for the attacher

const std::string ID_1 = "Hexagon";
const std::string ID_2 = "Hexagon_2";
const std::string ID_3 = "Hexagon_3";

const std::string ID_4 = "cube";
const std::string ID_5 = "cube_5";
const std::string ID_6 = "cube_6";

const std::string ID_7 = "Triangle";
const std::string ID_8 = "Triangle_8";
const std::string ID_9 = "Triangle_9";

#endif