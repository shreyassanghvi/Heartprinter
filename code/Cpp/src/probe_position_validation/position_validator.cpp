#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <stdexcept>

// Precision tolerance for floating point comparisons
// Adjust this value to make the test more or less strict
// Smaller values = more strict, larger values = more tolerant of numerical errors
const double PRECISION_TOLERANCE = 1e-4;

struct Probe {
    double x, y, z;
    double r, theta, phi;

    Probe() : x(0), y(0), z(0), r(0), theta(0), phi(0) {}
    
    Probe(double x_pos, double y_pos, double z_pos, 
          double radius, double th, double ph) 
        : x(x_pos), y(y_pos), z(z_pos), r(radius), theta(th), phi(ph) {}
    
    // Constructor that parses a line from the text file
    explicit Probe(const std::string& line) {
        std::istringstream iss(line);
        std::string dash;
        
        if (!(iss >> x >> y >> z >> r >> theta >> phi)) {
            throw std::invalid_argument("Invalid line format: " + line);
        }
    }

    // Calculate distance to another probe
    double distanceTo(const Probe& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    // Print probe information
    void print() const {
        std::cout << x << " " << y << " " << z << " " 
                  << r << " " << theta << " " << phi << std::endl;
    }
};

class Configuration {
public:
    enum ProbeType { L = 0, C = 1, R = 2, MV = 3, NUM_PROBES = 4 };

private:
    std::vector<Probe> probes; // All probes in order: L, C, R, MV
    
    // Check if point is inside triangle using area-based method with projection
    bool isPointInTriangle(const Probe& p, const Probe& a, const Probe& b, const Probe& c) const {
        // Step 1: Project point P onto the plane containing triangle ABC
        
        // Find the plane normal vector using cross product of two triangle edges
        double ab_x = b.x - a.x, ab_y = b.y - a.y, ab_z = b.z - a.z;  // Vector AB
        double ac_x = c.x - a.x, ac_y = c.y - a.y, ac_z = c.z - a.z;  // Vector AC
        
        // Normal vector = AB × AC
        double normal_x = ab_y * ac_z - ab_z * ac_y;
        double normal_y = ab_z * ac_x - ab_x * ac_z;
        double normal_z = ab_x * ac_y - ab_y * ac_x;
        
        // Normalize the normal vector
        double normal_length = sqrt(normal_x*normal_x + normal_y*normal_y + normal_z*normal_z);
        if (normal_length < PRECISION_TOLERANCE) {
            return false; // Degenerate triangle (collinear points)
        }
        normal_x /= normal_length;
        normal_y /= normal_length;
        normal_z /= normal_length;
        
        // Find the projection of P onto the plane
        // Distance from P to plane = (P-A) · normal
        double ap_x = p.x - a.x, ap_y = p.y - a.y, ap_z = p.z - a.z;
        double distance_to_plane = ap_x * normal_x + ap_y * normal_y + ap_z * normal_z;
        
        // Projected point = P - distance * normal
        double proj_x = p.x - distance_to_plane * normal_x;
        double proj_y = p.y - distance_to_plane * normal_y;
        double proj_z = p.z - distance_to_plane * normal_z;
        
        // Step 2: Calculate areas using the projected point
        
        // Area of original triangle ABC
        double area_abc = triangleArea(a, b, c);
        
        // Areas of three sub-triangles formed by projected point
        Probe proj_point(proj_x, proj_y, proj_z, 0, 0, 0);
        double area_pab = triangleArea(proj_point, a, b);
        double area_pbc = triangleArea(proj_point, b, c);
        double area_pca = triangleArea(proj_point, c, a);
        
        // Step 3: Check if sum of sub-triangle areas equals the original triangle area
        double sum_of_sub_areas = area_pab + area_pbc + area_pca;
        double area_difference = std::abs(sum_of_sub_areas - area_abc);
        
        // Point is inside triangle if the areas match within tolerance
        // If P is inside: area(PAB) + area(PBC) + area(PCA) = area(ABC)
        // If P is outside: area(PAB) + area(PBC) + area(PCA) > area(ABC)
        return area_difference <= PRECISION_TOLERANCE * area_abc;
    }
    
    // Calculate the area of a triangle using cross product
    double triangleArea(const Probe& a, const Probe& b, const Probe& c) const {
        double x1 = a.x, y1 = a.y, z1 = a.z;
        double x2 = b.x, y2 = b.y, z2 = b.z;
        double x3 = c.x, y3 = c.y, z3 = c.z;
        
        // Vector AB and AC
        double ab_x = x2 - x1, ab_y = y2 - y1, ab_z = z2 - z1;
        double ac_x = x3 - x1, ac_y = y3 - y1, ac_z = z3 - z1;
        
        // Cross product AB x AC
        double cross_x = ab_y * ac_z - ab_z * ac_y;
        double cross_y = ab_z * ac_x - ab_x * ac_z;
        double cross_z = ab_x * ac_y - ab_y * ac_x;
        
        return 0.5 * sqrt(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
    }

public:
    Configuration() : probes(NUM_PROBES) {}
    
    bool parseFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open file " << filename << std::endl;
            return false;
        }
        
        std::string line;
        int lineIndex = 0; 
        
        while (std::getline(file, line) && lineIndex < NUM_PROBES) {
            if (line.empty()) continue;
            
            try {
                probes[lineIndex] = Probe(line);
                lineIndex++;
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error: " << e.what() << std::endl;
                return false;
            }
        }
        
        file.close();
        
        if (lineIndex != NUM_PROBES) {
            std::cerr << "Error: Expected exactly " << NUM_PROBES 
                      << " probes. Found " << lineIndex << std::endl;
            return false;
        }
        
        return true;
    }
    
    bool validatePosition() const {        
        std::cout << "=== Validating Position ===" << std::endl;
        std::cout << "L - "; probes[L].print();
        std::cout << "C - "; probes[C].print();  
        std::cout << "R - "; probes[R].print(); 
        std::cout << "MV - "; probes[MV].print(); 
        std::cout << std::endl;
        
        // Check if the fixed probes form a valid triangle (non-collinear)
        double area = triangleArea(probes[L], probes[C], probes[R]);
        if (area < 1e-10) {
            std::cerr << "Error: Fixed probes are collinear - cannot form a valid triangle" << std::endl;
            return false;
        }
        
        // Main validation: Check if MV is inside the triangle
        //bool insideTriangle = isPointInTriangleCrossProduct(probes[MV], probes[L], probes[C], probes[R]);
        bool insideTriangle = isPointInTriangle(probes[MV], probes[L], probes[C], probes[R]);
        
        std::cout << "=== Result ===" << std::endl;
        
        if (insideTriangle) {
            std::cout << "VALID: Moving platform is within the triangle formed by L, C, and R" << std::endl;
        } else {
            std::cout << "INVALID: Moving platform is outside the triangle formed by L, C, and R" << std::endl;
        }

        std::cout << std::endl; 
        
        return insideTriangle;
    }
    
    void printConfiguration() const {
        std::cout << "=== Current Configuration ===" << std::endl;
        std::cout << "Fixed Probes:" << std::endl;
        for (int i = L; i <= R; i++) {
            probes[i].print();
        }
        std::cout << "Moving Platform:" << std::endl;
        probes[MV].print();
    }
};

int main(int argc, char* argv[]) {
    std::string filename = "config.txt";
    
    if (argc > 1) {
        filename = argv[1];
    }
    
    std::cout << "Heartprinter Position Validator" << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << "Reading configuration from: " << filename << std::endl << std::endl;
    
    Configuration config;
    
    if (!config.parseFromFile(filename)) {
        std::cerr << "Failed to parse configuration file" << std::endl;
        return 1;
    }
    
    bool isValid = config.validatePosition();
    
    return isValid ? 0 : 1;
}