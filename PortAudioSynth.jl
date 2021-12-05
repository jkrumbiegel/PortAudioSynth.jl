module PortAudioSynth

using PortAudio.LibPortAudio


mutable struct AudioBuffer
    ptr::Ptr{Float32}
    buffer::Vector{Float32} # vector reference so no problems with garbage collection
    len::Int
    pos::Ptr{Cint} # position of player in buffer
    posvec::Vector{Cint} # vector reference so no problems with garbage collection
end

function AudioBuffer(vec::Vector{Float32})
    posvec = Cint[1]
    AudioBuffer(
        Base.unsafe_convert(Ptr{Float32}, vec),
        vec, # for garbage safety
        length(vec),
        Base.unsafe_convert(Ptr{Cint}, posvec),
        posvec, # for garbage safety
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

mutable struct SineGenerator
    samplerate::Int
    phase::Float64
    frequency_hz::Float64
    volume::Float64
    current_period::Float64
end

function next_sample!(s::SineGenerator)::Float32
    delta_period = s.frequency_hz / s.samplerate * 2pi
    s.current_period += delta_period
    sample = s.volume * sin(s.current_period - s.phase)
end


function run(compute_next_sample)
    buflen = 4096
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
            nsamples = mod(playhead - lastplayhead, buflen)
            if nsamples <= 0
                yield()
                continue
            end
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
            yield()
        end
    finally
        @info "Stopping stream"
        @checkerr LibPortAudio.Pa_StopStream(pointer_to)
        @info "Terminating PortAudio"
        @checkerr LibPortAudio.Pa_Terminate()
    end
end

end