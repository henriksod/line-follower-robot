// Copyright (c) 2023 Henrik Söderlund

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "line_follower/service_agents/common/common.h"

namespace line_follower {
namespace {
struct ExampleData final {
    std::string message;
};

class ExampleDataProducerAgent : public ProducerAgent<ExampleData> {
 public:
    ExampleDataProducerAgent() = default;
    ~ExampleDataProducerAgent() noexcept = default;
};

class ExampleDataConsumerAgent : public ConsumerAgent<ExampleData> {
 public:
    ExampleDataConsumerAgent() = default;
    ~ExampleDataConsumerAgent() noexcept = default;
};

TEST(ConsumerProducerAgentTest, DoNotCrashWhenNotAttached) {
    ExampleDataProducerAgent producer;
    ExampleDataConsumerAgent consumer;

    ExampleData my_data{"This is my message!"};

    consumer.detach();
    producer.sendData(my_data);
}

TEST(ConsumerProducerAgentTest, SingleConsumer_ShouldReceiveWhenAttached) {
    ExampleDataProducerAgent producer;
    ExampleDataConsumerAgent consumer;

    ExampleData my_data{"This is my message!"};
    bool should_receive_data = false;

    consumer.onReceiveData([&](ExampleData const& data) {
        ASSERT_TRUE(my_data.message == data.message);
        ASSERT_TRUE(should_receive_data);
    });

    producer.sendData(my_data);

    consumer.attach(producer);
    should_receive_data = true;
    producer.sendData(my_data);

    consumer.detach();
    should_receive_data = false;
    producer.sendData(my_data);
}

TEST(ConsumerProducerAgentTest, MultipleConsumers_ShouldReceiveWhenAttached) {
    ExampleDataProducerAgent producer;
    ExampleDataConsumerAgent consumer_1;
    ExampleDataConsumerAgent consumer_2;

    ExampleData my_data{"This is my message!"};
    bool should_receive_data_1 = false;
    bool should_receive_data_2 = false;

    consumer_1.onReceiveData([&](ExampleData const& data) {
        ASSERT_TRUE(my_data.message == data.message);
        ASSERT_TRUE(should_receive_data_1);
    });

    consumer_2.onReceiveData([&](ExampleData const& data) {
        ASSERT_TRUE(my_data.message == data.message);
        ASSERT_TRUE(should_receive_data_2);
    });

    producer.sendData(my_data);

    consumer_1.attach(producer);
    should_receive_data_1 = true;
    producer.sendData(my_data);

    consumer_2.attach(producer);
    should_receive_data_2 = true;
    producer.sendData(my_data);

    consumer_1.detach();
    should_receive_data_1 = false;
    producer.sendData(my_data);

    consumer_2.detach();
    should_receive_data_2 = false;
    producer.sendData(my_data);
}

TEST(ConsumerProducerAgentTest, MultipleProducers_OnlyOneAttachedAtATime) {
    ExampleDataProducerAgent producer_1;
    ExampleDataProducerAgent producer_2;
    ExampleDataConsumerAgent consumer;

    ExampleData my_data_1{"This is my first message!"};
    ExampleData my_data_2{"This is my second message!"};
    bool should_receive_data = true;

    consumer.attach(producer_1);

    consumer.onReceiveData([&](ExampleData const& data) {
        ASSERT_TRUE(my_data_1.message == data.message);
        ASSERT_TRUE(should_receive_data);
    });

    producer_1.sendData(my_data_1);

    consumer.attach(producer_2);
    should_receive_data = false;
    producer_1.sendData(my_data_1);

    should_receive_data = true;
    consumer.onReceiveData([&](ExampleData const& data) {
        ASSERT_TRUE(my_data_2.message == data.message);
        ASSERT_TRUE(should_receive_data);
    });

    producer_2.sendData(my_data_2);
}
}  // namespace
}  // namespace line_follower
