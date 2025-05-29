#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <any>
#include <functional>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <numeric>

#include <catch2\catch_all.hpp>

using NodeId = int;

enum class NodeType
{
    None,
    Node6,
    Node7
};

struct IEdge
{
    virtual NodeId getDestination() const = 0;
    virtual std::any translate(const std::any&) = 0;
};

struct Edge : public IEdge {
    NodeId to;
    std::function<std::any(const std::any&)> transform;

    Edge(NodeId _to, std::function<std::any(const std::any&)> _transform)
        : to{ _to }
        , transform{ _transform }
    {}

    NodeId getDestination() const override
    {
        return to;
    }

    std::any translate(const std::any& v)
    {
        return transform(v);
    }
};

class Node {
public:
    std::function<std::any(const std::vector<std::any>&)> compute;
    std::vector<NodeId> inputFrom;
    std::any value;
    NodeType type{ NodeType::None };

    Node(std::function<std::any(const std::vector<std::any>&)> func, NodeType t) 
        : compute(func) 
        , type { t }
    {}
};

class Graph {
    std::unordered_map<NodeId, std::shared_ptr<Node>> nodes;
    std::unordered_map<NodeId, std::vector<std::shared_ptr<IEdge>>> forwardEdges;
    NodeId outputNode = -1;

public:
    void setOutput(NodeId id) { outputNode = id; }

    void addNode(
        NodeId id, 
        std::function<std::any(const std::vector<std::any>&)> computeFunc) 
    {
        nodes[id] = std::make_shared<Node>(computeFunc, NodeType::None);
    }

    void addNode(
        NodeId id, NodeType t,
        std::function<std::any(const std::vector<std::any>&)> computeFunc
        )
    {
        nodes[id] = std::make_shared<Node>(computeFunc, t);
    }

    void addEdge(NodeId from, NodeId to, std::function<std::any(const std::any&)> transform) {
        forwardEdges[from].push_back(std::make_shared<Edge>( to, transform ));
        nodes[to]->inputFrom.push_back(from);
    }


    //void propagate(std::any input) {
    //    if (rootNode == -1 || outputNode == -1) throw std::runtime_error("Root or output not set");

    //    // Inject root input
    //    nodes[rootNode]->value = input;

    //    // Topological order processing (Kahn's algorithm variant)
    //    std::unordered_map<NodeId, int> inDegree;
    //    std::queue<NodeId> ready;

    //    // Calculate in-degrees
    //    for (auto& [id, node] : nodes) {
    //        inDegree[id] = static_cast<int>(node->inputFrom.size());
    //    }

    //    // Start from root
    //    ready.push(rootNode);

    //    while (!ready.empty()) {
    //        NodeId current = ready.front();
    //        ready.pop();

    //        auto node = nodes[current];
    //        if (current != rootNode) {
    //            // Gather inputs
    //            std::vector<std::any> inputs;
    //            for (auto& from : node->inputFrom) {
    //                const auto& edges = forwardEdges[from];
    //                for (const auto& edge : edges) {
    //                    if (edge.to == current) {
    //                        inputs.push_back(edge.transform(nodes[from]->value));
    //                    }
    //                }
    //            }
    //            node->value = node->compute(inputs);
    //        }

    //        // Push value to children
    //        for (const auto& edge : forwardEdges[current]) {
    //            inDegree[edge.to]--;
    //            if (inDegree[edge.to] == 0) {
    //                ready.push(edge.to);
    //            }
    //        }
    //    }
    //}

    void propagateFrom(NodeId root, std::any input) {
        if (!nodes.count(root)) throw std::runtime_error("Root not found");

        nodes[root]->value = input;

        // In-degree count
        std::unordered_map<NodeId, int> inDegree;
        for (auto& [id, node] : nodes) {
            inDegree[id] = static_cast<int>(node->inputFrom.size());
        }

        std::queue<NodeId> ready;

        // Start with any node that has no inputs, or already has value
        for (auto& [id, node] : nodes) {
            if (inDegree[id] == 0 || node->value.has_value()) {
                ready.push(id);
            }
        }

        while (!ready.empty()) {
            NodeId current = ready.front();
            ready.pop();

            bool allReady = true;
            std::vector<std::any> inputs;
            auto& node = nodes[current];
            if (!node->value.has_value() && !node->inputFrom.empty()) {
                // Wait until all inputs are available
                
                
                for (NodeId from : node->inputFrom) {
                    if (!nodes[from]->value.has_value()) {
                        allReady = false;
                        break;
                    }

                    // Find transform function from edge
                    auto& edges = forwardEdges[from];
                    for (auto& edge : edges) {
                        if (edge->getDestination() == current) {
                            inputs.push_back(edge->translate(nodes[from]->value));
                            break;
                        }
                    }
                }

               
            }

            if (!allReady) continue;

            if (!node->value.has_value())
            {
                node->value = node->compute(inputs);
                std::cout << "after compute node: " << current << "\n";
            }

            // Push forward
            for (const auto& edge : forwardEdges[current]) {
                inDegree[edge->getDestination()]--;
                if (inDegree[edge->getDestination()] == 0) {
                    ready.push(edge->getDestination());
                }
            }
        }
    }

    std::any getOutput() const {
        return nodes.at(outputNode)->value;
    }
};

struct BruttoData
{
    int brutto;
    int bruttoWithExtra;
};

// Node operations
struct ComputeSkladka
{
    double operator()(const std::vector<std::any>& in)
    {
        int brutto = std::any_cast<int>(in[0]);
        double procent = std::any_cast<double>(in[1]);
        return brutto * procent;
    }
};

template <typename TData>
struct Const
{
    Const(const TData& data) : m_data{ data }
    {}

    double operator()(const std::vector<std::any>& in)
    {
        return m_data;
    }

private:
    TData m_data;
};

//Edge transformations
struct Identity
{
    std::any operator()(const std::any& v) 
    {
        return v;
    }
};

TEST_CASE("Computational graph example")
{
    //externale
    constexpr double procent_emerytalnej = 0.2;
    constexpr double procent_rentowej = 0.5;
    std::vector<int> components = { 3, 5, 7 };

    Graph g;


    //Dodawanie nodeów do grafu
    g.addNode(1, [](const std::vector<std::any>& in) { //brutto
        return in[0]; });

    g.addNode(2, [](const std::vector<std::any>& in) { //dodatkowe brutto jako suma sk³adników
        int brutto = std::any_cast<int>(in[0]);
        int extra = std::any_cast<int>(in[1]);
        return brutto + extra;});

    g.addNode(3, [](const std::vector<std::any>& in) { // sprawdzenie limitu (jakaœ zmienna steruj¹ca
        int brutto = std::any_cast<int>(in[0]);
        return brutto > 100; });

    g.addNode(4, [](const std::vector<std::any>& in) { // node wynikowy z regionu Brutto
        int brutto = std::any_cast<int>(in[0]);
        int bruttoWithExtra = std::any_cast<int>(in[1]);
        bool overlimit = std::any_cast<bool>(in[2]);

        return BruttoData {
            .brutto = overlimit ? brutto - 10 : brutto,
            .bruttoWithExtra = overlimit ? bruttoWithExtra - 10 : bruttoWithExtra
        }; 
        });

    g.addNode(5, [&components](const std::vector<std::any>&) { // suma extra sk³adnikow
        return std::accumulate(components.begin(), components.end(), 0); });

    g.addNode(6, NodeType::Node6, [](const std::vector<std::any>& in) { // input/root do nastepnego regionu SkladkiZUS
        return in[0]; }
        );

    g.addNode(7, NodeType::Node7, ComputeSkladka{});

    g.addNode(8, ComputeSkladka{});

    g.addNode(9, Const{procent_emerytalnej});

    g.addNode(10, [&](const std::vector<std::any>&) { //pobranie procentu emerytalnej
        return procent_rentowej;
        });

    g.addNode(11, [&](const std::vector<std::any>& in) { //agregacja skladek
        auto emerytalna = std::any_cast<double>(in[0]);
        auto rentowa = std::any_cast<double>(in[1]);
        return std::string("emerytalna = ") + std::to_string(emerytalna) 
             + std::string("\nrentowa = ") + std::to_string(rentowa);
        });


    //Ustalanie po³¹czeñ pomiêdzy grafami
    g.addEdge(1, 2, Identity{});
    g.addEdge(1, 3, Identity{});  
    g.addEdge(1, 4, Identity{});

    g.addEdge(2, 4, Identity{});

    g.addEdge(3, 4, Identity{});
    g.addEdge(4, 6, Identity{});

    g.addEdge(5, 2, Identity{});

    g.addEdge(6, 7, [](const std::any& v) { 
        BruttoData data = std::any_cast<BruttoData>(v);
        return data.brutto; });

    g.addEdge(6, 8, [](const std::any& v) { 
        BruttoData data = std::any_cast<BruttoData>(v);
        return data.bruttoWithExtra; 
        });

    g.addEdge(7, 11, Identity{});

    g.addEdge(8, 11, Identity{});
    
    g.addEdge(9, 7, Identity{});

    g.addEdge(10, 8, Identity{});

    g.setOutput(11);
    g.propagateFrom(1, 100);


   std::cout << std::any_cast<std::string>(g.getOutput()) << "\n";
}