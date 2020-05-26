#pragma once

template<typename T>
class resource_guard {
public:
    explicit resource_guard(T& resource) : resource(resource) {}

    ~resource_guard() {
        resource.reset();
    }
private:
    T& resource;
};

