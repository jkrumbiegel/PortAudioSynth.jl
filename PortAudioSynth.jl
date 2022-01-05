module PortAudioSynth

using PortAudio.LibPortAudio

export AudioEngine
export CombinedGenerator
export Delay
export Effect
export Gain
export PeriodicGenerator
export Track
export VolumeWobble
export start!
export stop!


mutable struct AudioBuffer
    ptr::Ptr{Float32}
    buffer::Vector{Float32} # reference to keep the pointer above valid
    len::Int
    readpos::Ptr{Cint} # position of reader in buffer
    readposvec::Vector{Cint} # reference to keep the pointer above valid
    writepos::Ptr{Cint} # position of writer in buffer
    writeposvec::Vector{Cint} # reference to keep the pointer above valid
    condition::Base.AsyncCondition
    condition_handle::Ptr{Nothing} # for calling with uv_async_send
end

function AudioBuffer(vec::Vector{Float32})
    readposvec = Cint[1]
    writeposvec = Cint[0]
    cond = Base.AsyncCondition()
    AudioBuffer(
        Base.unsafe_convert(Ptr{Float32}, vec),
        vec, # for garbage safety
        length(vec),
        Base.unsafe_convert(Ptr{Cint}, readposvec),
        readposvec, # for garbage safety
        Base.unsafe_convert(Ptr{Cint}, writeposvec),
        writeposvec, # for garbage safety
        cond,
        cond.handle,
    )
end


function portaudio_callback(inputbuffer, outputbuffer, framecount, timeinfo, statusflags, userdata)::Cint

    audio = unsafe_load(Ptr{AudioBuffer}(userdata))

    for i in 1:framecount
        # read new samples from ring buffer and if there are none available, write silence
        x = something(
            read_circ_buffer!(audio.ptr, audio.len, audio.readpos, audio.writepos),
            zero(Float32),
        )
        unsafe_store!(outputbuffer, x, 2i - 1)
        unsafe_store!(outputbuffer, x, 2i)
    end

    return 0
end

function write_circ_buffer!(bufptr, buflen, value, readposptr, writeposptr)
    readpos = unsafe_load(readposptr, 1)
    writepos = unsafe_load(writeposptr, 1)
    isfull = (writepos - readpos) + 1 == buflen
    if !isfull
        nextwritepos = writepos + 1
        unsafe_store!(bufptr, value, mod1(nextwritepos, buflen))
        Threads.atomic_fence() # ensure that the position is updated only after writing the value
        unsafe_store!(writeposptr, nextwritepos)
        return true
    end
    return false
end

function read_circ_buffer!(bufptr, buflen, readposptr, writeposptr)
    readpos = unsafe_load(readposptr, 1)
    writepos = unsafe_load(writeposptr, 1)
    isempty = writepos < readpos
    if !isempty
        value = unsafe_load(bufptr, mod1(readpos, buflen))
        Threads.atomic_fence() # ensure that the position is updated only after reading the value
        unsafe_store!(readposptr, readpos + 1)
        return value
    end
    return nothing
end

cfunc = @cfunction portaudio_callback Cint (
        Ptr{Float32},
        Ptr{Float32},
        Culong,
        Ptr{LibPortAudio.PaStreamCallbackTimeInfo},
        LibPortAudio.PaStreamCallbackFlags,
        Ptr{Cvoid}
    )

macro checkerr(exp)
    quote
        errcode = $(esc(exp))
        e = LibPortAudio.PaErrorCode(errcode)
        if e != LibPortAudio.paNoError
            error("PortAudio errored with status code $errcode ($(string(e)))")
        end
    end
end

mutable struct PeriodicGenerator{F}
    func::F
    samplerate::Int
    phase::Float64
    frequency_hz::Float64
    volume::Float64
    current_period::Float64
end

function next_sample!(s::PeriodicGenerator, t)::Float32
    delta_period = s.frequency_hz / s.samplerate * 2pi
    s.current_period = mod(s.current_period + delta_period, 2pi)
    sample = s.volume * s.func(s.current_period - s.phase)
end

abstract type Effect end

mutable struct Gain <: Effect
    gain::Float32
end

next_sample!(g::Gain, s, t) = g.gain * s

mutable struct Delay <: Effect
    samplerate::Int
    delay::Float64
    gain::Float64
    buffer::Vector{Float32}
    position::Int
end

function Delay(samplerate, delay, gain)
    bufferlength = round(Int, samplerate * delay)
    Delay(samplerate, delay, gain, zeros(Float32, bufferlength), 1)
end

function next_sample!(d::Delay, s::Float32, t)::Float32
    newpos = mod1(d.position + 1, length(d.buffer))
    x = d.buffer[newpos]
    y = d.buffer[newpos] = d.gain * s + d.gain * x
    d.position = newpos
    return y
end

mutable struct Track{T}
    generator::T
    effects::Vector{<:Effect}
end

mutable struct AudioEngine
    tracks::Vector{Track}
    bufferlength::Int
    framecount::Int
    samplerate::Int
    t::Float64
    buffer::Vector{Float32}
    _audio::AudioBuffer
    should_stop::Bool
end

function AudioEngine(;
        framecount = 256,
        samplerate = 44100,
        bufferlength = samplerate ÷ 2,
    )

    buffer = zeros(Float32, bufferlength)

    AudioEngine(
        [],
        bufferlength,
        framecount,
        samplerate,
        0.0,
        buffer,
        AudioBuffer(buffer),
        false,
    )
end


function start!(a::AudioEngine)
    @info "Initializing PortAudio"
    @checkerr LibPortAudio.Pa_Initialize()
    mutable_pointer = Ref{Ptr{LibPortAudio.PaStream}}(0)

    @checkerr LibPortAudio.Pa_OpenDefaultStream(
        mutable_pointer,
        0,
        2,
        LibPortAudio.paFloat32,
        a.samplerate,
        a.framecount,
        cfunc,
        Ref(a._audio),
    )

    pointer_to = mutable_pointer[]

    # the distance in samples that the writer should maintain against the reader
    dist = 2 * a.framecount

    try
        # as a heuristic, we trigger the timer at half the portaudio buffer duration
        timer = Timer(0.0, interval = 0.5 * a.framecount / a.samplerate) do _timer

            writepos = a._audio.writeposvec[1]
            readpos = a._audio.readposvec[1]
            # we always try to stay a specific distance ahead of the reader,
            # but the larger this is, the more audible delay there will be
            samples_to_write = max(0, dist - min(dist, writepos - readpos))

            if a.should_stop
                close(_timer)
                println("Timer stopped")
                return
            end
            # dt = time() - t_start
            # n_samples = round(Int, dt * a.samplerate)
            # n_new_samples = n_samples - n_samples_so_far
            # n_samples_so_far = n_samples
            for i in 1:samples_to_write
                new_sample = next_sample!(a)
                write_circ_buffer!(
                    a._audio.ptr,
                    a._audio.len,
                    new_sample,
                    a._audio.readpos,
                    a._audio.writepos,
                )
            end
        end

        @info "Starting stream"
        @checkerr LibPortAudio.Pa_StartStream(pointer_to)

        # keep this thing alive
        while !a.should_stop
            sleep(1/10)
        end

    finally
        @info "Stopping stream"
        @checkerr LibPortAudio.Pa_StopStream(pointer_to)
        @info "Terminating PortAudio"
        @checkerr LibPortAudio.Pa_Terminate()
    end
end

function stop!(a::AudioEngine)
    a.should_stop = true
end

function next_sample!(a::AudioEngine)::Float32
    a.t += 1/a.samplerate
    isempty(a.tracks) ? 0f0 : sum(next_sample!(t, a.t) for t in a.tracks)
end

mutable struct VolumeWobble <: Effect
    strength::Float64
    samplerate::Int
    phase::Float64
    frequency_hz::Float64
    current_period::Float64
end

function next_sample!(e::Track, t)::Float32
    s = next_sample!(e.generator, t)
    for ef in e.effects
        s::Float32 = next_sample!(ef, s, t)
    end
    s
end

function next_sample!(v::VolumeWobble, s, t)::Float32
    delta_period = v.frequency_hz / v.samplerate * 2pi
    v.current_period = mod(v.current_period + delta_period, 2pi)
    sample = s * (0.5 * v.strength * sin(v.current_period - v.phase) + (1 - 0.5 * v.strength))
end

struct CombinedGenerator{T<:Tuple}
    generators::T
end

CombinedGenerator(args...) = CombinedGenerator(tuple(args...))

function next_sample!(c::CombinedGenerator, t)::Float32
    sum((next_sample!(g, t) for g in c.generators), init = 0f0)
end

function triangle(period)
    if 0 <= period < 0.5pi
        period / 0.5pi
    elseif 0.5pi <= period < 1.5pi
        1 - (period - 0.5pi) / 0.5pi
    else
        (period - 2pi) / 0.5pi
    end
end

end