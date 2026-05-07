#include <cstdlib>
#include <iostream>

#include "hakoniwa/drone/control_adapter/feedback_policy_examples.hpp"
#include "hakoniwa/drone/control_adapter/px4_allocation_feedback_policy.hpp"

using namespace hakoniwa::drone::control_adapter;
using namespace hakoniwa::drone::control_adapter::test;

namespace {

class FeedbackPolicyExampleContext final : public IFeedbackPolicyExampleContext {
public:
    IAllocationFeedbackPolicy& feedback_policy() override
    {
        return policy_;
    }

    void reset_all() override
    {
        policy_.reset();
    }

private:
    Px4AllocationFeedbackPolicy policy_{};
};

}  // namespace

int main()
{
    try {
        FeedbackPolicyExampleContext context;
        return run_feedback_policy_examples(context);
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
}
