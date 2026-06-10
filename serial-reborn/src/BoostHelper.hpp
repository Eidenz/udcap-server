//
// Created by max_3 on 2025/5/9.
//

#ifndef SERIALREBORN_BOOSTHELPER_HPP
#define SERIALREBORN_BOOSTHELPER_HPP
#include <boost/asio.hpp>
#undef max
#undef min
#include <chrono>
#include <utility>
#include <mutex>

namespace aio = boost::asio;

template<typename Socket, typename Clock = std::chrono::steady_clock>
class DeadlineSocket{
public:
    DeadlineSocket(aio::io_context& ios)
            : mService(ios) {
        mTimer.expires_at(std::chrono::time_point<Clock>::max());
        check_timeout();
    }

    auto& socket() { return mSocket; }

    auto& io() { return mService; }

    template<typename Buffer, typename Duration, typename ... ErrorCode>
    size_t read(Buffer const& buf, Duration d, ErrorCode& ... ec) {
        static_assert(sizeof...(ec) <= 1, "only 0 or 1 error code allowed");
        size_t transfered = 0;
        with_timeout(d, [&buf, &transfered](auto& socket, auto& ec)
        {
            aio::async_read(socket, buf, [&ec, &transfered](auto err,auto transfer){ ec = err; transfered = transfer; });
        }, ec...);
        return transfered;
    }

    template<typename Buffer, typename Duration, typename ... ErrorCode>
    size_t read_some(Buffer const& buf, Duration d, ErrorCode& ... ec) {
        static_assert(sizeof...(ec) <= 1, "only 0 or 1 error code allowed");
        size_t transfered = 0;
        with_timeout(d, [&buf, &transfered](auto& socket, auto& ec)
        {
            socket.async_read_some(buf, [&ec, &transfered](auto err, auto transfer){ ec = err; transfered = transfer; });
        }, ec...);
        return transfered;
    }

    template<typename Duration, typename MatchCriteria, typename ... ErrorCode>
    size_t read_until(aio::streambuf& sb, MatchCriteria match, Duration d, ErrorCode& ... ec) {
        static_assert(sizeof...(ec) <= 1, "only 0 or 1 error code allowed");
        size_t transfered = 0;
        with_timeout(d, [&sb, &match, &transfered](auto& socket, auto& ec)
        {
            aio::async_read_until(socket, sb, match, [&ec, &transfered](auto err, auto transfer){ec = err; transfered = transfer; });
        }, ec...);
        return transfered;
    }

    template<typename Buffer, typename Duration, typename ... ErrorCode>
    size_t write(Buffer const& buf, Duration d, ErrorCode& ... ec) {
        static_assert(sizeof...(ec) <= 1, "only 0 or 1 error code allowed");
        size_t transfered = 0;
        with_timeout(d, [&buf, &transfered](auto& socket, auto& ec)
        {
            aio::async_write(socket, buf, [&ec, &transfered](auto err, auto transfer){ ec = err; transfered = transfer; });
        }, ec...);
        return transfered;
    }
private:
    using Timer = aio::basic_waitable_timer<Clock>;
    std::mutex ioMutex;

    template<typename Duration, typename Op>
    auto with_timeout(Duration d, Op&& op, boost::system::error_code& ec) {
        mTimer.expires_after(d);
        ec = aio::error::would_block;
        op(mSocket, ec);
        do mService.run_one(); while(ec == aio::error::would_block);
    }

    template<typename Duration, typename Op>
    auto with_timeout(Duration d, Op&& op) {
        std::lock_guard lk(ioMutex);
        boost::system::error_code ec;
        with_timeout(d, std::forward<Op>(op), ec);
        if(ec) throw boost::system::system_error(ec);
    }

    void check_timeout() {
        if(mTimer.expiry() <= Clock::now()){
            mSocket.cancel();
            mTimer.expires_at(std::chrono::time_point<Clock>::max());
        }
        mTimer.async_wait([this](auto&&...){ this->check_timeout(); });
    }

    aio::io_context& mService;

    Socket mSocket{mService};
    Timer mTimer{mService};
};

typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

std::pair<iterator, bool> match_whitespace(iterator begin, iterator);

template<typename F>
struct ConditionMet : private F{
    ConditionMet(F&& f) : F{std::forward<F>(f)} {}

    using F::operator();
};

namespace boost{ namespace asio {
    template <typename T> struct is_match_condition<ConditionMet<T>> : std::true_type{};
}}


#endif //SERIALREBORN_BOOSTHELPER_HPP
