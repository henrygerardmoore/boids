#include <math.h>
#include <SFML/Graphics.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include <vector>
#include <memory>
#include <random>

std::random_device dev;
std::mt19937 rng(dev());
std::uniform_int_distribution<std::mt19937::result_type> color_sampler(0,4294967295); // sample a uint324294967296
std::uniform_real_distribution angle_sampler(0., 2 * M_PI); // sample a random angle

template<typename T>
T DegToRad(T val) {
  static_assert(std::is_floating_point<T>::value, "DegToRad must be used on floating point types");
  return val * M_PI / 180.;
}

template<typename T>
T RadToDeg(T val) {
  static_assert(std::is_floating_point<T>::value, "RadToDeg must be used on floating point types");
  return val * 180. / M_PI;
}

struct State {
  float x;
  float y;
  float o;
};

struct Rates {
  float speed;
  float omega; // angular velocity
};

float distance(State s1, State s2) {
  return sqrt(pow(s1.x - s2.x, 2) + pow(s1.y - s2.y, 2));
}

float const BOID_SEPARATION_WEIGHT = 1;
float const BOID_ALIGNMENT_WEIGHT = 1;
float const BOID_COHESION_WEIGHT = 1;

class Boid {
  public:
    Boid(State s) : s_(s){
      shape_ = sf::ConvexShape();
      shape_.setPointCount(4);
      shape_.setPoint(0, sf::Vector2f(-1, 0));
      shape_.setPoint(1, sf::Vector2f(0, -3));
      shape_.setPoint(2, sf::Vector2f(1, 0));
      shape_.setPoint(3, sf::Vector2f(0, 1));
      shape_.setFillColor(sf::Color(color_sampler(rng))); // set this boid to a random color :)
      shape_.setScale(10,10);
    }
    [[nodiscard]] sf::Shape const& GetShape() {
      shape_.setPosition(s_.x, s_.y);
      shape_.setRotation(RadToDeg(s_.o) + 90);
      return shape_;
    }

    [[nodiscard]] State const& GetState() {
      return s_;
    }

    void SetState(State s) {
      s_ = s;
    }

    Rates GetRates() {
      return Rates{speed_, turn_rate_};
    }

    void UpdateRates(std::vector<std::shared_ptr<Boid>> const& boids) {
      auto const N = boids.size();
      sf::Vector2f separation_vector{0, 0};
      sf::Vector2f cohesion_vector{0, 0};
      sf::Vector2f other_boid_position;
      float theta_desired_alignment = s_.o;
      sf::Vector2f const this_boid_position{s_.x, s_.y};
      std::size_t boids_seen = 0;
      for (auto const& boid : boids) {
        if (boid.get() == this) {
          // skip ourselves
          continue;
        }
        auto const other_boid_state = boid->GetState();
        auto const d = distance(other_boid_state, s_);
        if (d > vision_distance_) {
          // skip if we can't see this boid
          continue;
        }
        ++boids_seen;

        other_boid_position.x = other_boid_state.x;
        other_boid_position.y = other_boid_state.y;

        // separation
        separation_vector += 1 / d * (this_boid_position - other_boid_position);

        // alignment
        theta_desired_alignment += other_boid_state.o;

        // cohesion
        cohesion_vector += other_boid_position - this_boid_position;
      }

      if (boids_seen == 0) {
        turn_rate_ = 0;
        speed_ = avg_speed_;
        return;
      }

      // normalize
      separation_vector /= float(boids_seen);
      theta_desired_alignment /= float(boids_seen);
      cohesion_vector /= float(boids_seen);

      // sum contributions for desired rotation rate
      auto const theta_desired_separation = atan2(separation_vector.y, separation_vector.x);
      auto const theta_desired_cohesion = atan2(cohesion_vector.y, cohesion_vector.x);
      float const theta_change = (BOID_SEPARATION_WEIGHT * theta_desired_separation + BOID_ALIGNMENT_WEIGHT * theta_desired_alignment + BOID_COHESION_WEIGHT * theta_desired_cohesion) / (BOID_SEPARATION_WEIGHT + BOID_ALIGNMENT_WEIGHT + BOID_COHESION_WEIGHT) - s_.o;
      
      turn_rate_ = std::min(std::max(-max_turn_rate, theta_change), max_turn_rate); // clip turn rate to [-max_turn_rate, max_turn_rate]


      // calculate desired speed
      auto const desired_position = (BOID_SEPARATION_WEIGHT * separation_vector + BOID_COHESION_WEIGHT * cohesion_vector) / (BOID_SEPARATION_WEIGHT + BOID_COHESION_WEIGHT);
      auto const angle_to_desired_position = atan2(desired_position.y - s_.y, desired_position.x - s_.x) - s_.o;
      auto const distance_to_desired_position = sqrt(pow(desired_position.y - s_.y, 2) + pow(desired_position.x - s_.x, 2));

      speed_ = cos(angle_to_desired_position) * distance_to_desired_position / vision_distance_ * max_speed_diff_ + avg_speed_; // if the desired location is as far as we can see directly behind us, go min speed; If it's as far as we can see in front of us, go max speed
      speed_ = std::min(std::max(avg_speed_ - max_speed_diff_, speed_), avg_speed_ + max_speed_diff_);
    }
  private:
    State s_;
    sf::ConvexShape shape_;
    float vision_distance_ = 100; // pixels
    float const max_turn_rate = M_PI; // radians / s
    float turn_rate_ = 0;
    float avg_speed_ = 50; // pixels / s
    float speed_ = avg_speed_; // pixels / s
    float const max_speed_diff_ = 20; // pixels / s
};


int main() {
  auto window = sf::RenderWindow{{1920u, 1080u}, "Boids"};
  unsigned int const desired_framerate = 165;
  float dt = 1 / float(desired_framerate);
  bool paused = false;

  window.setFramerateLimit(desired_framerate);
  std::vector<std::shared_ptr<Boid>> boids;
  while (window.isOpen()) {
    for (auto event = sf::Event{}; window.pollEvent(event);) {
      if (event.type == sf::Event::Closed) {
        window.close();
      } else if(event.type == sf::Event::MouseButtonPressed) {
        auto const coords = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        boids.emplace_back(std::make_shared<Boid>(State(coords.x, coords.y, angle_sampler(rng))));
      } else if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::Key::Space) {
          paused = !paused;
        }
      }
    }
    window.clear();
    for (auto const& boid : boids) {
      auto const& state = boid->GetState();
      boid->UpdateRates(boids);
      if (!paused) {
        auto const rates = boid->GetRates();
        auto const turn_rate = rates.omega;
        auto const boid_speed = rates.speed;
        auto const new_o = state.o + dt * turn_rate;
        auto const new_x = state.x + cos(state.o) * boid_speed * dt;
        auto const new_y = state.y + sin(state.o) * boid_speed * dt;

        boid->SetState(State{new_x, new_y, new_o});
      }

      window.draw(boid->GetShape());
    }
    window.display();
    
  }
}