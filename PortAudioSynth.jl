module PortAudioSynth

using PortAudio.LibPortAudio


mutable struct AudioBuffer
    ptr::Ptr{Float32}
    buffer::Vector{Float32} # vector reference so no problems with garbage collection
    len::Int
    pos::Ptr{Cint} # position of player in buffer
    posvec::Vector{Cint} # vector reference so no problems with garbage collection
    condition::Base.AsyncCondition
    condition_handle::Ptr{Nothing} # for calling with uv_async_send
end

function AudioBuffer(vec::Vector{Float32})
    posvec = Cint[1]
    cond = Base.AsyncCondition()
    AudioBuffer(
        Base.unsafe_convert(Ptr{Float32}, vec),
        vec, # for garbage safety
        length(vec),
        Base.unsafe_convert(Ptr{Cint}, posvec),
        posvec, # for garbage safety
        cond,
        cond.handle,
    )
end

function mycallback(inputbuffer, outputbuffer, framecount, timeinfo, statusflags, userdata)::Cint

    audio = unsafe_load(Ptr{AudioBuffer}(userdata))
    ptr = audio.ptr
    pos = unsafe_load(audio.pos, 1)

    for i in 1:256
        x = unsafe_load(ptr, pos - 1 + i)
        unsafe_store!(outputbuffer, x, 2i - 1)
        unsafe_store!(outputbuffer, x, 2i)
    end

    newpos = mod1(pos + 256, audio.len)
    unsafe_store!(audio.pos, newpos, 1)

    @ccall uv_async_send(audio.condition_handle::Ptr{Nothing})::Cvoid

    return 0
end

cfunc = @cfunction mycallback Cint (
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

function next_sample!(s::PeriodicGenerator)::Float32
    delta_period = s.frequency_hz / s.samplerate * 2pi
    s.current_period = mod(s.current_period + delta_period, 2pi)
    sample = s.volume * s.func(s.current_period - s.phase)
end


function run(compute_next_sample)
    buflen = 256 * 8
    framecount = 256
    samplerate = 44100
    buffer = zeros(Float32, buflen)
    audio = AudioBuffer(buffer)

    @info "Initializing PortAudio"
    @checkerr LibPortAudio.Pa_Initialize()
    mutable_pointer = Ref{Ptr{LibPortAudio.PaStream}}(0)

    @checkerr LibPortAudio.Pa_OpenDefaultStream(
        mutable_pointer,
        0,
        2,
        LibPortAudio.paFloat32,
        samplerate,
        framecount,
        cfunc,
        Ref(audio),
    )

    pointer_to = mutable_pointer[]
    @info "Starting stream"
    @checkerr LibPortAudio.Pa_StartStream(pointer_to)

    try
        lastplayhead = 1

        should_stop = false
        # this is cpu hogging
        while !should_stop
            playhead = Int(audio.posvec[1])
            # this will be wrong if the playhead went 1 cycle or more
            nsamples = mod(playhead - lastplayhead, buflen)
            pos = lastplayhead
            lastplayhead = playhead
            for i in 1:nsamples
                new_sample = compute_next_sample()
                if new_sample === nothing
                    should_stop = true
                    break
                end
                buffer[pos] = new_sample
                pos = mod1(pos + 1 , buflen)
            end
            wait(audio.condition)
        end
    finally
        @info "Stopping stream"
        @checkerr LibPortAudio.Pa_StopStream(pointer_to)
        @info "Terminating PortAudio"
        @checkerr LibPortAudio.Pa_Terminate()
    end
end

abstract type Effect end

mutable struct EffectChain{T}
    generator::T
    effects::Vector{<:Effect}
end

function next_sample!(e::EffectChain)::Float32
    s = next_sample!(e.generator)
    for ef in e.effects
        s = next_sample!(ef, s)
    end
    s
end

mutable struct VolumeWobble <: Effect
    strength::Float64
    samplerate::Int
    phase::Float64
    frequency_hz::Float64
    current_period::Float64
end

function next_sample!(e::EffectChain)::Float32
    s = next_sample!(e.generator)
    for ef in e.effects
        s::Float32 = next_sample!(ef, s)
    end
    s
end

function next_sample!(v::VolumeWobble, s)::Float32
    delta_period = v.frequency_hz / v.samplerate * 2pi
    v.current_period = mod(v.current_period + delta_period, 2pi)
    sample = s * (0.5 * v.strength * sin(v.current_period - v.phase) + (1 - 0.5 * v.strength))
end

struct CombinedGenerator{T<:Tuple}
    generators::T
end

CombinedGenerator(args...) = CombinedGenerator(tuple(args...))

function next_sample!(c::CombinedGenerator)::Float32
    sum((next_sample!(g) for g in c.generators), init = 0f0)
end

end