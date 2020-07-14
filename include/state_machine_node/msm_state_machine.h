#ifndef MSM_STATE_MACHINE_H
#define MSM_STATE_MACHINE_H

// back-end
#include <boost/msm/back/state_machine.hpp>
//front-end
#include <boost/msm/front/state_machine_def.hpp>


#define RESET   "\033[0m"
#define MAGENTA "\033[35m"
#define BOLDGREEN   "\033[1m\033[32m"

namespace msm = boost::msm;
namespace mpl = boost::mpl;

// events
struct ros_service_start_event {};
struct ros_service_close_event {};
struct udp_start_ack_recv_event {};

// front-end: define the FSM structure
struct msm_front_ : public msm::front::state_machine_def<msm_front_>
{
    template <class Event,class FSM>
    void on_entry(Event const& ,FSM&)
    {
        //std::cout << "Entering State" << std::endl;
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        //std::cout << "Leaving State" << std::endl;
    }

    // The list of FSM states
    struct Closed : public msm::front::state<>
    {
        // every (optional) entry/exit methods get the event passed.
        template <class Event,class FSM>
        void on_entry(Event const&,FSM& ) {std::cerr << "entering: " << BOLDGREEN << "Closed\n" << RESET;}
        template <class Event,class FSM>
        void on_exit(Event const&,FSM& )
        {
            //std::cerr << "leaving: Closed\n\n";
        }
    };
    struct StartSend : public msm::front::state<>
    {
        template <class Event,class FSM>
        void on_entry(Event const& ,FSM&)
        {
            std::cerr << "entering: " << BOLDGREEN << "StartSend\n" << RESET;
            std::cerr << MAGENTA << "FAKE START SEND\n" << RESET;
        }
        template <class Event,class FSM>
        void on_exit(Event const&,FSM& )
        {
            //std::cerr << "leaving: StartSend\n\n";
        }
    };
    struct StartAck : public msm::front::state<>
    {
        template <class Event,class FSM>
        void on_entry(Event const& ,FSM&) {std::cerr << "entering: " << BOLDGREEN << "StartAck\n" << RESET;}
        template <class Event,class FSM>
        void on_exit(Event const&,FSM& )
        {
            //std::cerr << "leaving: StartAck\n\n";
        }
    };

    // the initial state of the player SM. Must be defined
    typedef Closed initial_state;


    typedef msm_front_ m; // makes transition table cleaner

    // Transition table for player
    struct transition_table : mpl::vector<
        //    Start        Event                      Next      Action				 Guard
        //  +------------+--------------------------+---------+---------------------+----------------------+
        _row < Closed    , ros_service_start_event  , StartSend                                             >,
        _row < StartSend , udp_start_ack_recv_event , StartAck                                              >,
        _row < StartAck  , ros_service_close_event  , Closed                                                >
        //  +------------+--------------------------+---------+---------------------+----------------------+
    > {};
    // Replaces the default no-transition response.
    template <class FSM,class Event>
    void no_transition(Event const& e, FSM&,int state)
    {
        // IGNORE NON-EXISTENT TRANSITIONS FOR NOW
        std::cout << "no transition from state " << state << " on event " << typeid(e).name() << std::endl;
    }
};

// Pick a back-end
typedef msm::back::state_machine<msm_front_> protocol_state_machine;

#endif  //MSM_STATE_MACHINE_H