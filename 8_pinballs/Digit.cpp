#include <iostream>
#include "Digit.h"
#include "Segment.h"

// Create the seven segments locally
Digit::Digit(Eigen::Vector3d p) : pos(p) , digit(0) {
    segments.push_back(std::make_shared<Segment>(pos + Eigen::Vector3d(-0.5,2,0), true));
    segments.push_back(std::make_shared<Segment>(pos + Eigen::Vector3d(-1,1.5,0), false));
    segments.push_back(std::make_shared<Segment>(pos + Eigen::Vector3d(0,1.5,0), false));
    segments.push_back(std::make_shared<Segment>(pos + Eigen::Vector3d(-0.5,1,0), true));
    segments.push_back(std::make_shared<Segment>(pos + Eigen::Vector3d(-1,0.5,0), false));
    segments.push_back(std::make_shared<Segment>(pos + Eigen::Vector3d(0,0.5,0), false));
    segments.push_back(std::make_shared<Segment>(pos + Eigen::Vector3d(-0.5,0,0), true));
}

//Update the digit to number d
void Digit::update_digit(int d) {

    digit = d;

    switch (digit) { // Determine which segments are on
        case 0: allTrue(bitmap); bitmap[3] = false; break;
        case 1: allFalse(bitmap); bitmap[2] = true; bitmap[5] = true; break;
        case 2: allTrue(bitmap); bitmap[1] = false; bitmap[5] = false; break;
        case 3: allTrue(bitmap); bitmap[1] = false; bitmap[4] = false; break;
        case 4: allTrue(bitmap); bitmap[0] = false; bitmap[4] = false; bitmap[6] = false; break;
        case 5: allTrue(bitmap); bitmap[2] = false; bitmap[4] = false; break;
        case 6: allTrue(bitmap); bitmap[2] = false; break;
        case 7: allFalse(bitmap); bitmap[0] = true; bitmap[2] = true; bitmap[5] = true; break;
        case 8: allTrue(bitmap); break;
        case 9: allTrue(bitmap); bitmap[4] = false; break;
    }
    // Apply them to the segments
    for(int i = 0; i < 7; i++) {
        segments[i]->toggleSegment(bitmap[i]);
    }
}

void Digit::print_digit() {
    std::cout << (bitmap[0] ? " -" : "  ") << std::endl <<
    (bitmap[1] ? "|" : " ") << " " << (bitmap[2] ? "|" : " ") << std::endl <<
    (bitmap[3] ? " -" : "  ") << std::endl <<
    (bitmap[4] ? "0" : " ") << " " << (bitmap[5] ? "|" : " ") << std::endl <<
    (bitmap[6] ? " -" : "  ") << std::endl;
} 

void Digit::allTrue(bool bm[]) {
    for(int i = 0; i < 7; i++) bm[i] = true;
}

void Digit::allFalse(bool bm[]) {
    for(int i = 0; i < 7; i++) bm[i] = false;
}

void Digit::emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj) {
    for(auto s : segments) m_obj->emplace_back(s);
}

void Digit::resetDigit() {
    digit = 0;
    segments.at(0)->resetSegment(true);
    segments.at(1)->resetSegment(true);
    segments.at(2)->resetSegment(true);
    segments[3]->resetSegment(false);
    segments[4]->resetSegment(true);
    segments[5]->resetSegment(true);
    segments[6]->resetSegment(true);
}