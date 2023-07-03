#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <SFML/Graphics.hpp>
#include <spdlog/spdlog.h>

std::random_device Dev;
std::mt19937 RNG(Dev());
std::uniform_int_distribution<std::mt19937::result_type> ColorSampler(
    0, 4294967295);  // sample a uint324294967296
std::uniform_real_distribution<float> AngleSampler(
    0.f,
    float(2.f * M_PI));  // sample a random angle

float BoidAverageSpeed = 50;  // pixels / s

template <typename T>
T DegToRad(T val) {
  static_assert(std::is_floating_point<T>::value,
                "DegToRad must be used on floating point types");
  return val * M_PI / 180.;
}

template <typename T>
T RadToDeg(T val) {
  static_assert(std::is_floating_point<T>::value,
                "RadToDeg must be used on floating point types");
  return val * 180. / M_PI;
}

float FixAngle(float o) {
  auto fixed_angle = float(std::fmod(o, 2 * M_PI));
  if (fixed_angle > M_PI) {
    fixed_angle -= 2 * M_PI;
  }
  return fixed_angle;
}

struct State {
  float x;
  float y;
  float o;
  State(float x_pos, float y_pos, float o_pos, sf::Window const& window) {
    o = FixAngle(o_pos);
    auto const size = window.getSize();
    x = float(std::fmod(x_pos, size.x));
    if (x < 0) {
      x += float(size.x);
    }
    y = float(std::fmod(y_pos, size.y));
    if (y < 0) {
      y += float(size.y);
    }
  }
};

struct Rates {
  float speed;
  float omega;  // angular velocity
};

float Distance(State s1, State s2) {
  return float(std::sqrt(std::pow(s1.x - s2.x, 2) + std::pow(s1.y - s2.y, 2)));
}

float const kBOID_SEPARATION_WEIGHT = 1;
float const kBOID_ALIGNMENT_WEIGHT = 1;
float const kBOID_COHESION_WEIGHT = 1;

class Boid {
 public:
  explicit Boid(State s) : s_(s) {
    shape_ = sf::ConvexShape();
    shape_.setPointCount(4);
    shape_.setPoint(0, sf::Vector2f(-1, 0));
    shape_.setPoint(1, sf::Vector2f(0, -3));
    shape_.setPoint(2, sf::Vector2f(1, 0));
    shape_.setPoint(3, sf::Vector2f(0, 1));
    shape_.setFillColor(
        sf::Color(ColorSampler(RNG)));  // set this boid to a random color :)
    shape_.setScale(10, 10);
  }
  [[nodiscard]] sf::Shape const& getShape() {
    shape_.setPosition(s_.x, s_.y);
    shape_.setRotation(RadToDeg(s_.o) + 90);  // visual offset for shape
    return shape_;
  }

  [[nodiscard]] State const& getState() { return s_; }

  void setState(State const s) { s_ = s; }

  Rates getRates() { return Rates{speed_, turn_rate_}; }

  void updateRates(std::vector<std::shared_ptr<Boid>> const& boids) {
    sf::Vector2f separation_vector{0, 0};
    sf::Vector2f cohesion_vector{0, 0};
    sf::Vector2f other_boid_position;
    float theta_desired_alignment = 0;
    sf::Vector2f const this_boid_position{s_.x, s_.y};
    std::size_t boids_seen = 0;
    for (auto const& boid : boids) {
      if (boid.get() == this) {
        // skip ourselves
        continue;
      }
      auto const other_boid_state = boid->getState();
      auto const d = Distance(other_boid_state, s_);
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
      speed_ = BoidAverageSpeed;
      return;
    }

    // normalize
    separation_vector /= float(boids_seen);
    theta_desired_alignment /= float(boids_seen);
    cohesion_vector /= float(boids_seen);

    // sum contributions for desired rotation rate
    auto const theta_desired_separation =
        std::atan2(separation_vector.y, separation_vector.x);
    auto const theta_desired_cohesion =
        std::atan2(cohesion_vector.y, cohesion_vector.x);
    auto const theta_change =
        FixAngle((kBOID_SEPARATION_WEIGHT * theta_desired_separation +
                  kBOID_ALIGNMENT_WEIGHT * theta_desired_alignment +
                  kBOID_COHESION_WEIGHT * theta_desired_cohesion) /
                     (kBOID_SEPARATION_WEIGHT + kBOID_ALIGNMENT_WEIGHT +
                      kBOID_COHESION_WEIGHT) -
                 s_.o);

    turn_rate_ = std::min(
        std::max(-max_turn_rate_, theta_change),
        max_turn_rate_);  // clip turn rate to [-max_turn_rate_, max_turn_rate_]

    // calculate desired speed
    auto const desired_position =
        (kBOID_SEPARATION_WEIGHT * separation_vector +
         kBOID_COHESION_WEIGHT * cohesion_vector) /
        (kBOID_SEPARATION_WEIGHT + kBOID_COHESION_WEIGHT);
    auto const angle_to_desired_position = FixAngle(
        std::atan2(desired_position.y - s_.y, desired_position.x - s_.x) -
        s_.o);

    auto const distance_to_desired_position =
        float(std::sqrt(std::pow(desired_position.y - s_.y, 2) +
                        std::pow(desired_position.x - s_.x, 2)));

    speed_ =
        std::cos(angle_to_desired_position) * distance_to_desired_position /
            vision_distance_ * max_speed_diff_ +
        BoidAverageSpeed;  // if the desired location is as far as we can see
                           // directly behind us, go min speed; If it's as far
                           // as we can see in front of us, go max speed
    speed_ = std::min(std::max(BoidAverageSpeed - max_speed_diff_, speed_),
                      BoidAverageSpeed + max_speed_diff_);
  }

 private:
  State s_;
  sf::ConvexShape shape_;
  float vision_distance_ = 100;       // pixels
  float const max_turn_rate_ = M_PI;  // radians / s
  float turn_rate_ = 0;
  float speed_ = BoidAverageSpeed;   // pixels / s
  float const max_speed_diff_ = 20;  // pixels / s
};

int main() {
  auto window = sf::RenderWindow{{1920u, 1080u}, "Boids"};
  unsigned int const desired_framerate = 165;
  float dt = 1 / float(desired_framerate);
  bool paused = false;
  float turn_bias = 0;

  window.setFramerateLimit(desired_framerate);
  std::vector<std::shared_ptr<Boid>> boids;
  while (window.isOpen()) {
    for (auto event = sf::Event{}; window.pollEvent(event);) {
      if (event.type == sf::Event::Closed) {
        window.close();
      } else if (event.type == sf::Event::MouseButtonPressed) {
        auto const coords =
            window.mapPixelToCoords(sf::Mouse::getPosition(window));
        boids.emplace_back(std::make_shared<Boid>(
            State(coords.x, coords.y, AngleSampler(RNG), window)));
      } else if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::Key::Space) {
          paused = !paused;
        } else if (event.key.code == sf::Keyboard::Key::R) {
          boids.clear();
        } else if (event.key.code == sf::Keyboard::Key::Equal) {
          BoidAverageSpeed += 10;
          BoidAverageSpeed = std::min(300.f, BoidAverageSpeed);
        } else if (event.key.code == sf::Keyboard::Key::Dash) {
          BoidAverageSpeed -= 10;
          BoidAverageSpeed = std::max(10.f, BoidAverageSpeed);
        } else if (event.key.code == sf::Keyboard::Key::Left) {
          turn_bias = -1;
        } else if (event.key.code == sf::Keyboard::Key::Right) {
          turn_bias = 1;
        }
      } else if (event.type == sf::Event::KeyReleased) {
        if (event.key.code == sf::Keyboard::Left ||
            event.key.code == sf::Keyboard::Right) {
          turn_bias = 0;
        }
      }
    }
    window.clear();
    for (auto const& boid : boids) {
      auto const& state = boid->getState();
      boid->updateRates(boids);
      if (!paused) {
        auto const rates = boid->getRates();
        auto const turn_rate = rates.omega;
        auto const boid_speed = rates.speed;
        auto const new_o = state.o + dt * (turn_rate + turn_bias);
        auto const new_x = state.x + std::cos(state.o) * boid_speed * dt;
        auto const new_y = state.y + std::sin(state.o) * boid_speed * dt;

        auto const wrapped_state = State(new_x, new_y, new_o, window);

        boid->setState(wrapped_state);
      }

      window.draw(boid->getShape());
    }
    window.display();
  }
}
