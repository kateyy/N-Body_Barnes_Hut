/**
 * std::vector-style data structure that does not destruct contained values on
 * clear() and resize() to smaller sizes.
 *
 * This way, discarded values are caches for new values. Especially interesting
 * for composite data types, e.g., class Foo with a std::vector member. The data
 * of the vector can be reused instead of dynamically allocating it on each
 * insert.
 * Memory reuse is possible when using using push_back/emplace/emplace_back:
 * These functions will initialize the current "back" item using the
 * T::*initializeFunc (second and third template parameter).
 * NOTE: An actual copy/move of a whole object into a specific position in the
 * vector is only possible via [] operators/iterator access, not using
 * push_back/emplace/emplac_back!
 *
 * When removing values, a T::*clearFunc is called (forth template parameter).
 * This function must make sure that the deleted values have some consistent
 * state, which is recognized as "cleared".
 *
 * Example instantiation:
 *  PersistentVector<Node,
 *      void(Node::*)(int, float, double), &Node::initialize,
 *      &Node::clear> nodeVector;
 * Arguments are: the value type, the template of the initializer function,
 * the member function pointer of the initializer, and the member function
 * pointer of the clear function.
 *
 */
template<typename ValueType,
    typename InitFuncT, InitFuncT,
    void(ValueType::*)(void)> class PersistentVector;
template<typename ValueType,
    typename ...InitFuncArgs, void(ValueType::*initializeFunc)(InitFuncArgs...),
    void(ValueType::*clearFunc)(void)>
class PersistentVector<ValueType,
    void(ValueType::*)(InitFuncArgs...), initializeFunc, clearFunc>
{
public:
    using Data_t = std::vector<ValueType>;
    using iterator = typename Data_t::iterator;
    using const_iterator = typename Data_t::iterator;

    PersistentVector()
        : m_size{ 0 }
        , m_data{}
    {
    }
    ~PersistentVector() noexcept = default;

    iterator begin() noexcept {
        return m_data.begin();
    }
    const_iterator begin() const noexcept {
        return m_data.begin();
    }
    iterator end() noexcept {
        return m_data.begin() + m_size;
    }
    const_iterator end() const noexcept {
        return m_data.begin() + m_size;
    }

    operator const Data_t&() const noexcept {
        return m_data;
    }

    void reserve(const size_t size) {
        if (size <= m_size) {
            return;
        }
        m_data.resize(size);
    }
    void resize(const size_t size) {
        m_size = size;
        // Don't free memory of overhanging values, just "clear"
        for (size_t i = size; i < m_data.size(); ++i) {
            (m_data[i].*clearFunc)();
        }
    }
    size_t reservedSize() const noexcept {
        return m_data.size();
    }
    size_t size() const noexcept {
        return m_size;
    }
    void push_back(const ValueType &item) {
        if (m_size == m_data.size()) {
            m_data.push_back(item);
        } else {
            assert(m_size < m_data.size());
            (m_data[m_size].*initializeFunc)(item);
        }
        ++m_size;
    }
    void push_back(ValueType &&item) {
        emplace_back(std::move(item));
    }
    template<typename... Params>
    void emplace_back(Params&&... params) {
        if (m_size == m_data.size()) {
            m_data.emplace_back(std::forward<Params>(params)...);
        } else {
            assert(m_size < m_data.size());
            (m_data[m_size].*initializeFunc)(std::forward<Params>(params)...);
        }
        ++m_size;
    }
    ValueType& operator[](const size_t index) noexcept {
        assert(index < m_size);
        return m_data[index];
    }
    const ValueType& operator[](const size_t index) const noexcept {
        assert(index < m_size);
        return m_data[index];
    }
    ValueType& front() noexcept {
        return operator[](0u);
    }
    const ValueType& front() const noexcept {
        return operator[](0u);
    }
    ValueType& back() noexcept {
        return operator[](m_size - 1);
    }
    const ValueType& back() const noexcept {
        return operator[](m_size - 1);
    }

private:
    size_t m_size;
    Data_t m_data;
};
