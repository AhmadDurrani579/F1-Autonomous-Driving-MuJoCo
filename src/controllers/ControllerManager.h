#pragma once

#include "controllers/DrivingController.h"

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class ControllerManager {
public:
    void add(std::unique_ptr<DrivingController> controller) {
        if (!controller) {
            throw std::invalid_argument("Cannot register a null controller");
        }
        controllers_.push_back(std::move(controller));
    }

    std::size_t count() const {
        return controllers_.size();
    }

    std::size_t selectedIndex() const {
        return selectedIndex_;
    }

    void select(std::size_t index) {
        if (index >= controllers_.size()) {
            throw std::out_of_range("Invalid controller index");
        }
        selectedIndex_ = index;
    }

    DrivingController* active() {
        return controllers_.empty()
            ? nullptr
            : controllers_[selectedIndex_].get();
    }

    const DrivingController* active() const {
        return controllers_.empty()
            ? nullptr
            : controllers_[selectedIndex_].get();
    }

    const char* name(std::size_t index) const {
        if (index >= controllers_.size()) {
            throw std::out_of_range("Invalid controller index");
        }
        return names_.at(index).c_str();
    }

    void addNamed(std::unique_ptr<DrivingController> controller) {
        if (!controller) {
            throw std::invalid_argument("Cannot register a null controller");
        }
        names_.push_back(controller->name());
        controllers_.push_back(std::move(controller));
    }

    void clear() {
        controllers_.clear();
        names_.clear();
        selectedIndex_ = 0;
    }

private:
    std::vector<std::unique_ptr<DrivingController>> controllers_;
    std::vector<std::string> names_;
    std::size_t selectedIndex_ = 0;
};
