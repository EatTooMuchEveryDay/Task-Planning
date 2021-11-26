///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Rui Xiao
//////////////////////////////////////

#define OPENCV

#include <iostream>
#include <set>
#include <queue>
#include <ctime>
#include <vector>
#include <fstream>
#include <unordered_map>

#include "c++/z3++.h"

#ifdef OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#endif

using namespace z3;

// config
#define MAX_ATTEMPT_STEP 20
#define VISUAL_CELL_SIZE 50

class InputBundle;
typedef std::shared_ptr<InputBundle> inputptr;
typedef std::pair<int, int> pair;
typedef std::vector<std::pair<int, int> > pairvec;
typedef std::set<std::pair<int, int> > pairset;
typedef std::vector<std::vector<bool> > board;
typedef std::vector<expr> varvec;
typedef std::pair<varvec, varvec> varvecpair;
typedef std::vector<std::vector<expr> > varboard;
typedef std::vector<std::vector<std::vector<expr> > > multivarboard;

static const std::string dirstr[] = {"UP", "RIGHT", "DOWN", "LEFT"};
static const int dirstep[4][2] = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};


// tool funciton
constexpr std::uint32_t hash_str_to_uint32(const char *data)
{
    std::uint32_t h(0);
    for (int i = 0; data && ('\0' != data[i]); i++)
        h = (h << 6) ^ (h >> 26) ^ data[i];
    return h;
}

std::vector<std::vector<int> > generateCombinations(int len, int n)
{
    std::vector<std::vector<int> > t(1, std::vector<int>()), ans(t);

    for (int idx = 0; idx < len; idx++)
    {
        t = {};

        for (auto i : ans)
        {
            if (i.size() < n)
            {
                i.push_back(idx + 1);
                t.push_back(i);
            }
        }

        ans.insert(ans.end(), t.begin(), t.end());
    }

    return ans;
}

template <class T>
const T &min(const T &a, const T &b)
{
    return a > b ? b : a;
}
template <class T>
const T &max(const T &a, const T &b)
{
    return a < b ? b : a;
}

class InputBundle
{
public:
    board obsboard;
    pairset boxes;
    pair start, goal, size;

    static inputptr GenerateFromFile(const char fpath[])
    {
        pairvec obs;
        pairset box;
        pair _size, _start, _goal;

        // setup file input stream
        std::ifstream is(fpath);
        if (!is.is_open())
        {
            std::cout << "Can't open the file!" << std::endl;
        }

        // fetch file information
        while (!is.eof())
        {
            std::string name;
            is >> name;

            switch (hash_str_to_uint32(name.data()))
            {
            case hash_str_to_uint32("size"):
                is >> _size.first >> _size.second;
                break;
            case hash_str_to_uint32("g"):
                is >> _goal.first >> _goal.second;
                break;
            case hash_str_to_uint32("r"):
                is >> _start.first >> _start.second;
                break;
            case hash_str_to_uint32("movable"):
                for (is.ignore(1); is.peek() >= '0' && is.peek() <= '9'; is.ignore(1))
                {
                    pair t;
                    is >> t.first >> t.second;
                    box.insert(t);
                }
                break;
            case hash_str_to_uint32("obstacle"):
                for (is.ignore(1); is.peek() >= '0' && is.peek() <= '9'; is.ignore(1))
                {
                    obs.push_back(pair());
                    is >> obs[obs.size() - 1].first >> obs[obs.size() - 1].second;
                }
                break;
            case hash_str_to_uint32("end"):
                break;
            default:
                std::cout << "Error in input format:" << std::endl
                          << name << std::endl;
                break;
            }
        }

        return std::make_shared<InputBundle>(obs, box, _size, _start, _goal);
    }

    InputBundle(const pairvec &obs, const pairset &box, const pair &_size, const pair &_start, const pair &_goal) : boxes(box), size(_size), start(_start), goal(_goal)
    {
        obsboard = board(_size.first, std::vector<bool>(_size.second, false));

        // setup static obstacles
        for (auto i : obs)
        {
            if (i.first < 0 || i.first >= _size.first || i.second < 0 || i.second >= _size.second)
            {
                throw "Static obstacle out of boundary: " + std::to_string(i.first) + "-" + std::to_string(i.second);
            }

            obsboard[i.first][i.second] = true;
        }

        // 0 sized game board is invalid
        if (size.first == 0 || size.second == 0)
        {
            std::cout << "Invalid size." << std::endl;
            return;
        }
    }
};

class Variables
{
public:
    // operations
    varvec opvec;

    // states
    varvecpair robboard;
    varboard boxboard;

    Variables(context &c, const inputptr input, int k)
    {
        // setup move operation propositions
        for (int i = 0; i < 4; i++)
        {
            opvec.push_back(c.bool_const(("OP_" + dirstr[i] + "_" + std::to_string(k)).data()));
        }

        // setup rob state propositions
        for (int x = 0; x < input->size.first; x++)
        {
            robboard.first.push_back(c.bool_const(("ROB_X_" + std::to_string(x) + "_" + std::to_string(k)).data()));
        }
        for (int y = 0; y < input->size.second; y++)
        {
            robboard.second.push_back(c.bool_const(("ROB_Y_" + std::to_string(y) + "_" + std::to_string(k)).data()));
        }

        // setup box state propositions
        for (int x = 0; x < input->size.first; x++)
        {
            boxboard.push_back(varvec(input->size.second, expr(c)));

            for (int y = 0; y < input->size.second; y++)
            {
                // obstacle cell
                if (input->obsboard[x][y])
                {
                    continue;
                }

                boxboard[x][y] = c.bool_const(("BOX_" + std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(k)).data());
            }
        }
    }
};

#ifdef OPENCV
enum celltype
{
    obstacle,
    robot,
    box,
    goal
};

void drawCell(cv::Mat &image, int x, int y, celltype type)
{
    cv::Scalar color;

    switch (type)
    {
    case celltype::obstacle:
        color = cv::Scalar(0, 0, 0);
        break;
    case celltype::robot:
        color = cv::Scalar(0, 0, 255);
        break;
    case celltype::box:
        color = cv::Scalar(255, 0, 0);
        break;
    case celltype::goal:
        color = cv::Scalar(0, 255, 0);
        break;
    }

    cv::rectangle(image, cvPoint(x * VISUAL_CELL_SIZE, image.size[0] - y * VISUAL_CELL_SIZE),
                  cvPoint((x + 1) * VISUAL_CELL_SIZE, image.size[0] - (y + 1) * VISUAL_CELL_SIZE),
                  color, cv::FILLED, 0);
}

void visualize(inputptr input, std::vector<Variables> &variables, solver &s, int step)
{
    model m = s.get_model();

    // init cv image and window
    cv::Mat originimg = cv::Mat(VISUAL_CELL_SIZE * input->size.second, VISUAL_CELL_SIZE * input->size.first, CV_8UC3, cv::Scalar(255, 255, 255));
    cvNamedWindow("solution", CV_WINDOW_AUTOSIZE);

    for (int x = 0; x < input->size.first; x++)
    {
        for (int y = 0; y < input->size.second; y++)
        {
            if (input->obsboard[x][y])
            {
                drawCell(originimg, x, y, celltype::obstacle);
            }
        }
    }
    drawCell(originimg, input->goal.first, input->goal.second, celltype::goal);

    for (int idx = 0; idx <= step; idx++)
    {
        cv::Mat img = originimg.clone();

        for (int x = 0; x < input->size.first; x++)
        {
            for (int y = 0; y < input->size.second; y++)
            {
                // obstacle cell
                if (input->obsboard[x][y])
                {
                    continue;
                }

                if (m.eval(variables[idx].robboard.first[x]).bool_value() == Z3_lbool::Z3_L_TRUE && m.eval(variables[idx].robboard.second[y]).bool_value() == Z3_lbool::Z3_L_TRUE)
                {
                    drawCell(img, x, y, celltype::robot);
                }
                if (m.eval(variables[idx].boxboard[x][y]).bool_value() == Z3_lbool::Z3_L_TRUE)
                {
                    drawCell(img, x, y, celltype::box);
                }
            }
        }

        // show the image in a window
        IplImage tmp = cvIplImage(img);
        cvShowImage("solution", static_cast<CvArr *>(&tmp));
        std::cout << "press any key to move" << std::endl;
        cvWaitKey(0);
    }
}
#endif

void solve(inputptr input)
{
    context c;
    solver s(c);

    std::vector<Variables> variables;

    // initial state
    variables.push_back(Variables(c, input, 0));
    // start rob pos
    for (int x = 0; x < input->size.first; x++)
    {
        if (x == input->start.first)
        {
            s.add(variables[0].robboard.first[x]);
        }
        else
        {
            s.add(!variables[0].robboard.first[x]);
        }
    }
    for (int y = 0; y < input->size.second; y++)
    {
        if (y == input->start.second)
        {
            s.add(variables[0].robboard.second[y]);
        }
        else
        {
            s.add(!variables[0].robboard.second[y]);
        }
    }
    // movable obstacles
    for (int x = 0; x < input->size.first; x++)
    {
        for (int y = 0; y < input->size.second; y++)
        {
            // obstacle cell
            if (input->obsboard[x][y])
            {
                continue;
            }

            if (input->boxes.find(pair(x, y)) != input->boxes.end())
            {
                s.add(variables[0].boxboard[x][y]);
            }
            else
            {
                s.add(!variables[0].boxboard[x][y]);
            }
        }
    }

    // start iteration
    for (int idx = 0; idx < MAX_ATTEMPT_STEP; idx++)
    {
        // generate a new set of variable
        variables.push_back(Variables(c, input, idx + 1));

        // operator encodings & framce axioms
        varvec opconjs;
        for (int i = 0; i < 4; i++)
        {
            opconjs.push_back(!variables[idx].opvec[i]);
        }

        // init frame axioms
        multivarboard robconjs(input->size.first, varboard(input->size.second));
        multivarboard boxconjs(input->size.first, varboard(input->size.second));
        for (int x = 0; x < input->size.first; x++)
        {
            for (int y = 0; y < input->size.second; y++)
            {
                // obstacle cell
                if (input->obsboard[x][y])
                {
                    continue;
                }

                // moved to x,y
                robconjs[x][y].push_back((variables[idx].robboard.first[x] && variables[idx].robboard.second[y]) || !(variables[idx + 1].robboard.first[x] && variables[idx + 1].robboard.second[y]));
                // moved away from x,y
                robconjs[x][y].push_back(!(variables[idx].robboard.first[x] && variables[idx].robboard.second[y]) || (variables[idx + 1].robboard.first[x] && variables[idx + 1].robboard.second[y]));

                // box moved to x,y
                boxconjs[x][y].push_back(variables[idx].boxboard[x][y] || !variables[idx + 1].boxboard[x][y]);
                // box moved away from x,y
                boxconjs[x][y].push_back(!variables[idx].boxboard[x][y] || variables[idx + 1].boxboard[x][y]);
            }
        }

        for (int x = 0; x < input->size.first; x++)
        {
            for (int y = 0; y < input->size.second; y++)
            {
                // obstacle cell
                if (input->obsboard[x][y])
                {
                    continue;
                }

                for (int dir = 0; dir < 4; dir++)
                {
                    int emptycount = 0;
                    pair nextstep(x + dirstep[dir][0], y + dirstep[dir][1]);
                    while (nextstep.first >= 0 && nextstep.first < input->size.first && nextstep.second >= 0 && nextstep.second < input->size.second && input->obsboard[nextstep.first][nextstep.second] == false)
                    {
                        nextstep.first += dirstep[dir][0];
                        nextstep.second += dirstep[dir][1];
                        emptycount++;
                    }

                    // no need to consider walking toward wall
                    if (emptycount == 0)
                    {
                        continue;
                    }

                    auto boxcomb = generateCombinations(emptycount, min<int>(emptycount - 1, (int)input->boxes.size()));

                    for (auto &i : boxcomb)
                    {
                        pair pos(x + dirstep[dir][0] * (emptycount - i.size()), y + dirstep[dir][1] * (emptycount - i.size()));

                        // init operation encodings
                        expr opconj = variables[idx].robboard.first[x] && variables[idx].robboard.second[y] && variables[idx + 1].robboard.first[pos.first] && variables[idx + 1].robboard.second[pos.second] && !(variables[idx + 1].robboard.first[x] && variables[idx + 1].robboard.second[y]);
                        // init frame axioms
                        expr frameconj = variables[idx].robboard.first[x] && variables[idx].robboard.second[y] && variables[idx].opvec[dir];

                        for (int j = 1; j <= emptycount; j++)
                        {
                            if (std::binary_search(i.begin(), i.end(), j))
                            {
                                opconj = opconj && variables[idx].boxboard[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j];
                                frameconj = frameconj && variables[idx].boxboard[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j];
                            }
                            else
                            {
                                opconj = opconj && !variables[idx].boxboard[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j];
                                frameconj = frameconj && !variables[idx].boxboard[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j];
                            }

                            if (j <= emptycount - i.size())
                            {
                                opconj = opconj && !variables[idx + 1].boxboard[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j];
                            }
                            else
                            {
                                opconj = opconj && variables[idx + 1].boxboard[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j];
                            }
                        }

                        // set operation encodings
                        opconjs[dir] = opconjs[dir] || opconj;

                        // set robot frame axioms
                        robconjs[x][y][1] = robconjs[x][y][1] || frameconj;
                        robconjs[pos.first][pos.second][0] = robconjs[pos.first][pos.second][0] || frameconj;

                        // set box frame axioms
                        for (int j = 1; j <= emptycount; j++)
                        {
                            if (std::binary_search(i.begin(), i.end(), j))
                            {
                                if (j <= emptycount - i.size())
                                {
                                    // box moved away
                                    boxconjs[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j][1] = boxconjs[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j][1] || frameconj;
                                }
                            }
                            else
                            {
                                if (j > emptycount - i.size())
                                {
                                    // box moved to here
                                    boxconjs[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j][0] = boxconjs[x + dirstep[dir][0] * j][y + dirstep[dir][1] * j][0] || frameconj;
                                }
                            }
                        }
                    }
                }
            }
        }

        for (auto &i : opconjs)
        {
            s.add(i);
        }
        for (int x = 0; x < input->size.first; x++)
        {
            for (int y = 0; y < input->size.second; y++)
            {
                // obstacle cell
                if (input->obsboard[x][y])
                {
                    continue;
                }

                s.add(robconjs[x][y][0]);
                s.add(robconjs[x][y][1]);

                s.add(boxconjs[x][y][0]);
                s.add(boxconjs[x][y][1]);
            }
        }

        // complete exclusion axiom
        for (int i = 0; i < 4; i++)
        {
            for (int j = i + 1; j < 4; j++)
            {
                s.add(!variables[idx].opvec[i] || !variables[idx].opvec[j]);
            }
        }

        // goal state
        s.push();
        s.add(variables[idx + 1].robboard.first[input->goal.first] && variables[idx + 1].robboard.second[input->goal.second]);

        // try to solve
        if (s.check() == sat)
        {
            // found!
            std::cout << "Solution found at " + std::to_string(idx + 1) + "th step!" << std::endl;
            model m = s.get_model();

            // statistics information
            std::cout << s.statistics() << std::endl;
            // print model
            std::cout << m << "\n";
#ifdef OPENCV
            // visualize the solution
            visualize(input, variables, s, idx + 1);
#endif

            return;
        }

        s.pop();
    }

    std::cout << "No solution found in " + std::to_string(MAX_ATTEMPT_STEP) + " steps." << std::endl;
}

struct State
{
public:
    pair rob;
    std::vector<pair> boxes;
    int depth;
    size_t hash;

    State(const pair &_rob, const std::vector<pair> &_boxes, int _depth) : rob(_rob), boxes(_boxes), depth(_depth)
    {
        std::hash<std::string> hashfn;
        std::string str = "ROB_" + std::to_string(rob.first) + "_" + std::to_string(rob.second);
        this->hash = hashfn(str);

        for (auto &i : boxes)
        {
            this->hash ^= hashfn("BOX_" + std::to_string(i.first) + "_" + std::to_string(i.second));
        }
    }
};

State simulateMove(int dir, inputptr input, State &state)
{
    int emptycount = 0;
    pair nextstep(state.rob.first + dirstep[dir][0], state.rob.second + dirstep[dir][1]);
    std::vector<int> affectedboxes;
    while (nextstep.first >= 0 && nextstep.first < input->size.first && nextstep.second >= 0 && nextstep.second < input->size.second && input->obsboard[nextstep.first][nextstep.second] == false)
    {
        for (int idx = 0; idx < state.boxes.size(); idx++)
        {
            if (nextstep == state.boxes[idx])
            {
                affectedboxes.push_back(idx);
            }
        }

        nextstep.first += dirstep[dir][0];
        nextstep.second += dirstep[dir][1];
        emptycount++;
    }

    pair rob = state.rob;
    pairvec boxes = state.boxes;
    rob = pair(rob.first + (emptycount - affectedboxes.size()) * dirstep[dir][0], rob.second + (emptycount - affectedboxes.size()) * dirstep[dir][1]);
    for (int idx = 0; idx < affectedboxes.size(); idx++)
    {
        boxes[affectedboxes[idx]] = pair(rob.first + dirstep[dir][0] * (idx + 1), rob.second + dirstep[dir][1] * (idx + 1));
    }

    return State(rob, boxes, state.depth + 1);
}

bool solve_BFS(inputptr input, bool memory = false)
{
    std::unordered_map<size_t, int> map;

    std::queue<State> queue;
    queue.push(State(input->start, pairvec(input->boxes.begin(), input->boxes.end()), 0));

    while (!queue.empty())
    {
        auto state = queue.front();
        queue.pop();

        if (memory)
        {
            if (map.find(state.hash) != map.end())
            {
                if (state.depth < map[state.hash])
                {
                    map[state.hash] = state.depth;
                }
                else
                {
                    continue;
                }
            }
        }

        if (state.rob == input->goal)
        {
            // solution found
            std::cout << "solution found in " + std::to_string(state.depth) + " steps!" << std::endl;
            return true;
        }

        if (state.depth < MAX_ATTEMPT_STEP)
        {
            for (int dir = 0; dir < 4; dir++)
            {
                auto nextstate = simulateMove(dir, input, state);
                if (nextstate.hash != state.hash)
                {
                    queue.push(nextstate);
                }
            }
        }
    }

    std::cout << "No solution found in " + std::to_string(MAX_ATTEMPT_STEP) + " steps." << std::endl;
    return false;
}

void benchmark(inputptr input)
{
    std::vector<double> times(50);

    for (int idx = 0; idx < times.size(); idx++)
    {
        clock_t start, end;
        start = clock();

        solve(input);
        // solve_BFS(input,false);

        end = clock();
        times[idx] = double(end - start) / CLOCKS_PER_SEC;

        Z3_finalize_memory();
    }

    // change to distribution
    times[0] = times.size() > 1 ? times[1] : times[0];
    std::sort(times.begin(), times.end());
    double min = times[0], length = times[times.size() - 1] - times[0];
    std::vector<double> distribution;
    int cut = times.size() / 4;
    for (int idx = 0; idx < cut; idx++)
    {
        int count = 0;
        for (auto i : times)
        {
            if (i <= min + length / cut * (idx + 1) && i >= min + length / cut * (idx))
            {
                count++;
            }
        }
        distribution.push_back(count);
    }
    std::cout << "distributions:" << std::endl;
    for (auto i : distribution)
    {
        std::cout << i << ",";
    }
    std::cout << std::endl;

    std::cout << "times:" << std::endl;
    for (auto i : times)
    {
        std::cout << i << ",";
    }
    std::cout << std::endl;
}

int main(int argc, const char *argv[])
{
    if (argc <= 1)
    {
        std::cout << "Please input the scene file like './Project5 filPath'" << std::endl;
        return -1;
    }

    // generate input environment
    auto input = InputBundle::GenerateFromFile(argv[1]);

    solve(input);
    // solve_BFS(input,true);
    // benchmark(input);

    Z3_finalize_memory();
    return 0;
}
