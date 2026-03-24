#include <rl_tools/operations/cpu_mux.h>

#include "../include/fixedwing/fixedwing.h"
#include "../include/fixedwing/operations_generic.h"
#include "../include/fixedwing/operations_cpu.h"

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn_models/operations_cpu.h>

#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>
#include <rl_tools/rl/algorithms/ppo/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/extrack/operations_cpu.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/save_trajectories/operations_cpu.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

using FIXEDWING_SPEC = FixedwingSpecification<T, TI, FixedwingParameters<T>>;
using ENVIRONMENT = Fixedwing<FIXEDWING_SPEC>;

struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::ppo::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    static constexpr TI N_ENVIRONMENTS = 16;
    static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 256;
    static constexpr TI BATCH_SIZE = 256;
    static constexpr TI TOTAL_STEP_LIMIT = 5000000;
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
    static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr TI STEP_LIMIT = TOTAL_STEP_LIMIT/(ON_POLICY_RUNNER_STEPS_PER_ENV * N_ENVIRONMENTS) + 1;
    static constexpr TI EPISODE_STEP_LIMIT = ENVIRONMENT::EPISODE_STEP_LIMIT;
    
    struct OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr T ALPHA = 0.0003;
    };
    
    static constexpr bool NORMALIZE_OBSERVATIONS = true;
    
    struct PPO_PARAMETERS: rlt::rl::algorithms::ppo::DefaultParameters<TYPE_POLICY, TI, BATCH_SIZE>{
        static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.01;
        static constexpr TI N_EPOCHS = 4;
        static constexpr T GAMMA = 0.99;
        static constexpr T INITIAL_ACTION_STD = 1.0;
    };
};

using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
#ifndef BENCHMARK
using LOOP_EXTRACK_CONFIG = rlt::rl::loop::steps::extrack::Config<LOOP_CORE_CONFIG>;

template <typename NEXT>
struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, NEXT>{
    static constexpr TI EVALUATION_INTERVAL = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / 10;
    static constexpr TI NUM_EVALUATION_EPISODES = 10;
    static constexpr TI N_EVALUATIONS = NEXT::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
};

using LOOP_EVALUATION_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_EXTRACK_CONFIG, LOOP_EVAL_PARAMETERS<LOOP_EXTRACK_CONFIG>>;

struct LOOP_SAVE_TRAJECTORIES_PARAMETERS: rlt::rl::loop::steps::save_trajectories::Parameters<TYPE_POLICY, TI, LOOP_EVALUATION_CONFIG>{
    static constexpr TI INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / 5;
    static constexpr TI INTERVAL = INTERVAL_TEMP == 0 ? 1 : INTERVAL_TEMP;
    static constexpr TI NUM_EPISODES = 10;
};

using LOOP_SAVE_TRAJECTORIES_CONFIG = rlt::rl::loop::steps::save_trajectories::Config<LOOP_EVALUATION_CONFIG, LOOP_SAVE_TRAJECTORIES_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_SAVE_TRAJECTORIES_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;

#else
using LOOP_CONFIG = LOOP_CORE_CONFIG;
#endif

using LOOP_STATE = typename LOOP_CONFIG::template State<LOOP_CONFIG>;

#include <chrono>
#include <iostream>

int main(){
    DEVICE device;
    TI seed = 42;
    LOOP_STATE ls;
    
#ifndef BENCHMARK
    ls.extrack_config.name = "fixedwing_velocity_tracking";
#endif
    
    rlt::malloc(device, ls);
    rlt::init(device, ls, seed);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::cout << "Starting Fixed-Wing Velocity Tracking Training..." << std::endl;
    std::cout << "Environment: " << LOOP_CORE_PARAMETERS::N_ENVIRONMENTS << " parallel environments" << std::endl;
    std::cout << "Total steps: " << LOOP_CORE_PARAMETERS::TOTAL_STEP_LIMIT << std::endl;
    std::cout << "Episode length: " << ENVIRONMENT::EPISODE_STEP_LIMIT << std::endl;
    
    while(!rlt::step(device, ls)){
        // Training loop - the loop interface handles everything
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    std::cout << "Training completed in: " << diff.count() << " s" << std::endl;
    
    return 0;
}
