#pragma once

class resource_guard {
public:
    explicit resource_guard(const std::function<void()>& resource_handler) : resource_handler(resource_handler) {}

    ~resource_guard() {
        resource_handler();
    }
private:
    const std::function<void()>& resource_handler;
};

