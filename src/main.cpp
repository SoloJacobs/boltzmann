#include <iostream>
#include <vector>
#include <array>
#include <algorithm>

struct Point_2D{
    double x;
    double y;
};

constexpr size_t Num_velocities = 4;

struct Element{
    std::array<bool, Num_velocities> hasVelocity;
    Point_2D coordinates;
    bool & North() {
        return hasVelocity[0];
    }
    bool & South() {
        return hasVelocity[1];
    }
    bool & West() {
        return hasVelocity[2];
    }
    bool & East() {
        return hasVelocity[3];
    }
};

Element collisionOperator(const Element& element)
{
    Element result = element;
    if (result.North() xor result.South()) {
        std::swap(result.North(), result.South());
    }
    if (result.West() xor result.East()) {
        std::swap(result.West(), result.East());
    }
    return result;
}

struct Grid{

    Grid(size_t column_length, size_t row_width) : row_width(row_width) {
        elements = std::vector<Element>((row_width+2) * (column_length+2));
        Element boundary{std::array<bool,4>{0, 0, 0, 0},Point_2D{0.,0.}};
        std::fill(elements.begin(), elements.begin() + row_width + 2, boundary);
        std::fill(elements.rbegin(), elements.rbegin() + row_width + 2, boundary);
        for(auto i=row_width + 2; i < elements.size(); i += row_width + 2)
        {
            elements[i] = boundary;
            elements[i-1] = boundary;
        }

    }

    enum class Direction{
        NORTH=0,
        SOUTH=1,
        WEST=2,
        EAST=3
    };

    size_t to_underlying(Direction d)
    {
        return static_cast<size_t>(d);
    }

    std::vector<Element> elements;
    size_t row_width;

    std::array<size_t, Num_velocities> streamingOperatorMask(size_t index){
        return { index - (row_width + 2), index + (row_width+2), index-1, index+1};
    }

    std::array<Element, Num_velocities> getMaskedElements(std::array<size_t, Num_velocities> mask)
    {
        std::array<Element, Num_velocities> result;
        std::transform(mask.begin(), mask.end(), result.begin(), [this](const auto index){return elements[index];});
        return result;
    }

    Element streamingOperator(size_t index, const std::vector<Element>& elements){
        Element result = elements[index];
        auto relevant_indices = streamingOperatorMask(index);
        auto relevant_elements = getMaskedElements(relevant_indices);
        result.North() = relevant_elements[to_underlying(Direction::SOUTH)].North();
        result.South() = relevant_elements[to_underlying(Direction::NORTH)].South();
        result.East() = relevant_elements[to_underlying(Direction::WEST)].East();
        result.West() = relevant_elements[to_underlying(Direction::EAST)].West();
        return result;
    }

    std::vector<double> getGrid() const{
        std::vector<double> result{};
        for(const auto& elem : elements)
        {
            double newVal = 0;
            for(const auto& velo : elem.hasVelocity)
            {
                newVal += velo ? 0.25 : 0.0;
            }
            result.emplace_back(newVal);
        }
        return result;
    }
};

void render(size_t width, size_t height, std::vector<double> values) {
    for(auto h=0; h<height; ++h)
    {
        for(auto w=0; w<width; ++w)
        {
            auto val = values[h*width + w];
            if(val > 0.5) {
                std::cout << 'o';
            } else {
                std::cout << ' ';
            }
        }
        std::cout << '\n';
    }
}

template<typename Result_container_t>
void plot(size_t width, size_t height, Result_container_t cont, double dt) {
    for(const auto& iteration : cont)
    {
        render(width, height, iteration);
        std::cout << "\n====\n";
    }
}


int main() {
    Grid test_grid(10, 10);
    return 0;
}
