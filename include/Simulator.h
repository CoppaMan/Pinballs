#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "Simulation.h"

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <mutex>
#include <thread>

using namespace std::chrono;

/*
 * Class for running a simulation (see Simulation.h) on a separate thread.
 * Performs stepping through simulation (including pausing/resuming/killing) and
 * updating the rendering in the viewer.
 */

class Simulator {
   public:
    Simulator(Simulation *sim)
        : p_simulator_thread(NULL),
          p_simulation(sim),
          m_please_pause(false),
          m_please_die(false),
          m_running(false),
          m_started(false) {}

    virtual ~Simulator() {
        killSimulatorThread();
        delete p_simulation;
        p_simulation = NULL;
    }

    /*
     * Runs the simulation, if it has been paused (or never started).
     */
    void run() {
        m_status_mutex.lock();
        if (!m_started) {
            p_simulation->reset();
            m_started = true;
            std::cout << "Start simulation" << std::endl;
        } else {
            std::cout << "Resume simulation" << std::endl;
        }
        m_please_pause = false;
        m_status_mutex.unlock();
    }

    /*
     * Resets the p_simulation (and leaves it in a paused state; call run() to
     * start it).
     */
    void reset() {
        killSimulatorThread();
        if (m_started) {
            std::cout << "Reset simulation" << std::endl;
        }
        m_please_die = m_running = m_started = false;
        m_please_pause = true;
        p_simulation->reset();
        p_simulation->updateRenderGeometry();
        p_simulator_thread = new std::thread(&Simulator::runSimThread, this);
    }

    /*
     * Pause a m_running p_simulation. The p_simulation will pause at the end of
     * its current "step"; this method will not interrupt simulateOneStep
     * mid-processing.
     */
    void pause() {
        m_status_mutex.lock();
        m_please_pause = true;
        m_status_mutex.unlock();
        std::cout << "Pause simulation" << std::endl;
    }

    bool isPaused() {
        bool ret = false;
        m_status_mutex.lock();
        if (m_running && m_please_pause) ret = true;
        m_status_mutex.unlock();
        return ret;
    }

    bool hasStarted() const { return m_started; }

    void render(igl::opengl::glfw::Viewer &viewer) {
        m_render_mutex.lock();
        p_simulation->renderRenderGeometry(viewer);
        m_render_mutex.unlock();
    }

    double getDuration() const {
        return duration_cast<microseconds>(m_duration).count() * 0.001;
    }

    double getSimulationTime() const { return p_simulation->getTime(); }

    unsigned long getSimulationStep() const { return p_simulation->getStep(); }

   protected:
    void runSimThread() {
        m_status_mutex.lock();
        m_running = true;
        m_status_mutex.unlock();

        bool done = false;
        while (!done) {
            m_status_mutex.lock();
            bool pausenow = m_please_pause;
            m_status_mutex.unlock();

            if (pausenow) {
                // don't use to much CPU time
                std::this_thread::sleep_for(milliseconds(10));
            } else {
                // time execution of one loop (advance + rendering)
                high_resolution_clock::time_point start =
                    high_resolution_clock::now();

                done = p_simulation->advance();

                m_render_mutex.lock();
                p_simulation->updateRenderGeometry();
                m_render_mutex.unlock();

                high_resolution_clock::time_point end =
                    high_resolution_clock::now();

                m_duration = end - start;

                // sleep such that simulation runs at approximately 60fps
                std::this_thread::sleep_for(
                    milliseconds(17) - duration_cast<milliseconds>(m_duration));
            }

            m_status_mutex.lock();
            if (m_please_die) done = true;
            m_status_mutex.unlock();
        }

        m_status_mutex.lock();
        m_running = false;
        m_status_mutex.unlock();
    }

    void killSimulatorThread() {
        if (p_simulator_thread) {
            m_status_mutex.lock();
            m_please_die = true;
            m_status_mutex.unlock();
            p_simulator_thread->join();
            delete p_simulator_thread;
            p_simulator_thread = NULL;
        }
    }

    std::thread *p_simulator_thread;
    Simulation *p_simulation;
    duration<double> m_duration;
    bool m_please_pause;
    bool m_please_die;
    bool m_running;
    bool m_started;
    std::mutex m_render_mutex;
    std::mutex m_status_mutex;
};

#endif