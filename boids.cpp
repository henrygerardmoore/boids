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
  float fixed_angle = o;
  if (fixed_angle < 0) {
    fixed_angle -= 2 * float(M_PI * (floor(fixed_angle / (2 * M_PI))));
  }
  if (fixed_angle > 2 * M_PI) {
    fixed_angle -= 2 * float(M_PI * (floor(fixed_angle / (2 * M_PI))));
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

float const kBOID_SEPARATION_WEIGHT =
    200;  // how far boids want to stay from one another
float const kBOID_ALIGNMENT_WEIGHT =
    1;  // how strongly boids want to align with one another
float const kBOID_COHESION_WEIGHT =
    0.1;  // how much boids want to go to the center of the flock
float const kBOID_ATTRACTION_WEIGHT =
    2;  // how much boids want to go toward the mouse cursor
float const kBOID_SPEED_FACTOR =
    0.05;  // how much boids will speed up or slow down
float const kBOID_MAX_SPEED_DIFF = 60;  // pixels / s
bool Attraction = true;

class Boid {
 public:
  explicit Boid(State s, std::shared_ptr<sf::RenderWindow> window)
      : s_(s), window_(std::move(window)) {
    shape_ = sf::ConvexShape();
    shape_.setPointCount(4);
    shape_.setPoint(0, sf::Vector2f(-1, 0));
    shape_.setPoint(1, sf::Vector2f(0, -3));
    shape_.setPoint(2, sf::Vector2f(1, 0));
    shape_.setPoint(3, sf::Vector2f(0, 1));
    shape_.setFillColor(
        sf::Color(ColorSampler(RNG) |
                  255u));  // set this boid to a random color with full alpha
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
    sf::Vector2f alignment_vector{0, 0};
    sf::Vector2f cohesion_vector{0, 0};
    sf::Vector2f other_boid_position;
    sf::Vector2f const this_boid_position{s_.x, s_.y};
    std::size_t boids_seen = 0;
    for (auto const& boid : boids) {
      if (boid.get() == this) {
        // skip ourselves
        continue;
      }
      auto const other_boid_state = boid->getState();

      other_boid_position.x = other_boid_state.x;
      other_boid_position.y = other_boid_state.y;
      auto const dp = other_boid_position - this_boid_position;

      auto const d = std::sqrt(dp.x * dp.x + dp.y * dp.y);

      if (d > vision_distance_) {
        // skip if we can't see this boid
        continue;
      }
      ++boids_seen;

      other_boid_position.x = other_boid_state.x;
      other_boid_position.y = other_boid_state.y;

      // separation
      separation_vector += 1 / (std::sqrt(d) + 1) * -(dp / d);

      // alignment
      alignment_vector += sf::Vector2f(std::cos(other_boid_state.o),
                                       std::sin(other_boid_state.o));

      // cohesion
      cohesion_vector += dp;
    }

    if (boids_seen == 0) {
      turn_rate_ = 0;
      speed_ = BoidAverageSpeed;
      return;
    }

    ++boids_seen;  // increment this to count the current boid in the weighted
                   // average

    // normalize and weight
    separation_vector *= kBOID_SEPARATION_WEIGHT / float(boids_seen);
    alignment_vector *= kBOID_ALIGNMENT_WEIGHT / float(boids_seen);
    cohesion_vector *= kBOID_COHESION_WEIGHT / float(boids_seen);

    sf::Vector2f mouse_coords_rel = {0, 0};
    if (Attraction) {
      // if enabled, calculate mouse attraction component
      auto const mouse_coords =
          window_->mapPixelToCoords(sf::Mouse::getPosition(*window_));

      auto const window_size = window_->getSize();

      // don't chase the mouse if it's outside the window
      if (window_->hasFocus() && 0 <= mouse_coords.x &&
          mouse_coords.x <= float(window_size.x) && 0 <= mouse_coords.y &&
          mouse_coords.y <= float(window_size.y)) {
        mouse_coords_rel = mouse_coords - this_boid_position;
      }
    }

    auto d_mouse = std::sqrt(mouse_coords_rel.x * mouse_coords_rel.x +
                             mouse_coords_rel.y * mouse_coords_rel.y);
    if (d_mouse == 0) {
      d_mouse = 1;
    }

    // calculate sum of vectors
    auto const desired_position_rel =
        separation_vector + alignment_vector + cohesion_vector +
        kBOID_ATTRACTION_WEIGHT * (mouse_coords_rel / d_mouse);

    // angle to desired position, relative to current angle
    auto desired_angle_rel =
        std::atan2(desired_position_rel.y, desired_position_rel.x) - s_.o;

    // if the angle wraps around, fix it
    if (desired_angle_rel > M_PI) {
      desired_angle_rel -= 2 * M_PI;
    } else if (desired_angle_rel < -M_PI) {
      desired_angle_rel += 2 * M_PI;
    }

    turn_rate_ = std::min(
        std::max(-max_turn_rate_, desired_angle_rel),
        max_turn_rate_);  // clip turn rate to [-max_turn_rate_, max_turn_rate_]

    auto const distance_to_desired_position =
        float(std::sqrt(std::pow(desired_position_rel.y, 2) +
                        std::pow(desired_position_rel.x, 2)));

    speed_ = kBOID_SPEED_FACTOR * std::cos(desired_angle_rel) *
                 distance_to_desired_position * kBOID_MAX_SPEED_DIFF +
             BoidAverageSpeed;  // move faster if we need to catch up, slower if
                                // we are ahead
    speed_ = std::min(std::max(BoidAverageSpeed - kBOID_MAX_SPEED_DIFF, speed_),
                      BoidAverageSpeed + kBOID_MAX_SPEED_DIFF);
  }

 private:
  State s_;
  sf::ConvexShape shape_;
  std::shared_ptr<sf::RenderWindow> window_;
  float vision_distance_ = 300;       // pixels
  float const max_turn_rate_ = M_PI;  // radians / s
  float turn_rate_ = 0;
  float speed_ = BoidAverageSpeed;  // pixels / s
};

int main() {
  auto const window = std::make_shared<sf::RenderWindow>(
      sf::VideoMode(1920u, 1080u), std::string("SFML"));
  unsigned int const desired_framerate = 165;
  float dt = 1 / float(desired_framerate);
  bool paused = false;
  float turn_bias = 0;

  window->setFramerateLimit(desired_framerate);
  std::vector<std::shared_ptr<Boid>> boids;
  while (window->isOpen()) {
    for (auto event = sf::Event{}; window->pollEvent(event);) {
      if (event.type == sf::Event::Closed) {
        window->close();
      } else if (event.type == sf::Event::MouseButtonPressed) {
        auto const coords =
            window->mapPixelToCoords(sf::Mouse::getPosition(*window));
        boids.emplace_back(std::make_shared<Boid>(
            State(coords.x, coords.y, AngleSampler(RNG), *window), window));
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
          BoidAverageSpeed = std::max(kBOID_MAX_SPEED_DIFF, BoidAverageSpeed);
        } else if (event.key.code == sf::Keyboard::Key::Left) {
          turn_bias = -1;
        } else if (event.key.code == sf::Keyboard::Key::Right) {
          turn_bias = 1;
        } else if (event.key.code == sf::Keyboard::Key::A) {
          Attraction = !Attraction;
        }
      } else if (event.type == sf::Event::KeyReleased) {
        if (event.key.code == sf::Keyboard::Left ||
            event.key.code == sf::Keyboard::Right) {
          turn_bias = 0;
        }
      }
    }
    window->clear();
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

        auto const wrapped_state = State(new_x, new_y, new_o, *window);

        boid->setState(wrapped_state);
      }

      window->draw(boid->getShape());
    }
    window->display();
  }
}
