# coding: utf-8

Gem::Specification.new do |spec|
  spec.name          = "ChoiRbot"
  spec.version       = "0.1.0"
  spec.authors       = ["Andrea Testa", "Andrea Camisa", "Giuseppe Notarstefano"]
  spec.email         = ["a.testa@unibo.it", "a.camisa@unibo.it", "giuseppe.notarstefano@unibo.it"]

  spec.summary       = %q{A ROS 2 Toolbox for Cooperative Robotics }
  spec.homepage      = "https://github.com/OPT4SMART/ChoiRbot"
  spec.license       = "MIT"

  spec.files         = `git ls-files -z`.split("\x0").select { |f| f.match(%r{^(_layouts|_includes|_sass|LICENSE|README)/i}) }

  spec.add_development_dependency "jekyll", "~> 3.2"
  spec.add_development_dependency "bundler", ">= 2.2.10"
  spec.add_development_dependency "rake", ">= 12.3.3"
end